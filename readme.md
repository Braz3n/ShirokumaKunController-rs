sudo apt install librust-openssl-dev
cargo install cargo-generate
git clone https://github.com/bentwire/embassy-rp2040-template.git 
cargo generate --path ./embassy-rp2040-template

# You then need to update all the versions to the most recent, and also change the channel in `rust-toolchain.toml`
# And then copy in the `cyw43-firmware` directory from the root of the embassy directory

`cargo run` to build and flash the target with a debug build. Add `--release` to specify a release build