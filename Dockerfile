FROM rust:latest as build

WORKDIR /usr/src
RUN rustup target add x86_64-unknown-linux-musl
RUN apt-get update -y && apt-get install -y musl-tools

WORKDIR /usr/src/file_format_conversions
COPY . .

RUN cargo build --release
RUN cargo install --target x86_64-unknown-linux-musl --path .

FROM scratch
COPY --from=build /usr/local/cargo/bin/file_format_conversions .
ENTRYPOINT ["./file_format_conversions"]

#FROM alpine:latest
#COPY --from=build /home/rust/target/release/file_format_conversions /usr/local/bin/file_format_conversions

#CMD ["file_format_conversions"]