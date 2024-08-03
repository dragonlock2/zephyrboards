use std::os::fd::AsRawFd;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_rustls::rustls;

/*
 * git clone https://github.com/rustls/rcgen.git && cd rcgen
 * cargo run -- --common-name=matthewtran.com --san=matthewtran.com --san=localhost --output=certs
 */

async fn connect_shell(addr: &str) -> tokio::io::Result<()> {
    // setup TLS
    let mut key = include_bytes!("../certs/cert.key.pem").as_slice();
    let mut cert = include_bytes!("../certs/cert.pem").as_slice();
    let key = rustls_pemfile::private_key(&mut key).unwrap().unwrap();
    let cert = rustls_pemfile::certs(&mut cert)
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    let cfg = rustls::ServerConfig::builder()
        .with_no_client_auth()
        .with_single_cert(cert, key)
        .unwrap();
    let acc = tokio_rustls::TlsAcceptor::from(std::sync::Arc::new(cfg));

    // start connection
    let tcp = tokio::net::TcpListener::bind(addr).await?;
    let (sock, addr) = tcp.accept().await?;
    let mut sock = acc.accept(sock).await?;
    println!("connected to {}", addr);

    // pipe to stdin/stdout
    let _ = tokio::spawn(async move {
        let mut stdin = tokio::io::stdin();
        let mut stdout = tokio::io::stdout();
        let mut read_buf = [0; 1024];
        let mut write_buf = [0; 1024];
        loop {
            tokio::select! {
                v = sock.read(&mut read_buf) => {
                    let v = v?;
                    match v {
                        0 => break,
                        len => {
                            stdout.write_all(&read_buf[0..len]).await?;
                            stdout.flush().await?
                        },
                    }
                }
                v = stdin.read(&mut write_buf) => {
                    let v = v?;
                    match v {
                        0 => break,
                        len => sock.write_all(&write_buf[0..len]).await?,
                    }
                }
            }
        }
        Ok::<(), tokio::io::Error>(())
    })
    .await;
    Ok(())
}

#[tokio::main]
async fn main() -> tokio::io::Result<()> {
    // disable stdin echo and newline buffering
    let stdin_fd = std::io::stdin().as_raw_fd();
    let termios_old = termios::Termios::from_fd(stdin_fd).unwrap();
    let mut termios = termios_old;
    termios.c_lflag &= !(termios::ECHO | termios::ICANON);
    termios.c_cc[termios::VMIN] = 1;
    termios.c_cc[termios::VTIME] = 0;
    termios::tcsetattr(stdin_fd, termios::TCSANOW, &termios)?;

    // start connection
    let addr = std::env::args()
        .nth(1)
        .unwrap_or("0.0.0.0:8080".to_string());
    let err = connect_shell(&addr).await;
    if let Err(msg) = err {
        println!("{}", msg);
    }

    // revert stdin
    termios::tcsetattr(stdin_fd, termios::TCSADRAIN, &termios_old)?;

    println!("press enter again to exit..."); // https://docs.rs/tokio/latest/tokio/io/struct.Stdin.html
    Ok(())

    // TODO manage multiple connections
}
