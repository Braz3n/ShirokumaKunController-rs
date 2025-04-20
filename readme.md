# aircon-rs
Rust implementation for controlling a Hitachi Shirokuma-kun air conditioner over IR.

## Starting the project from scratch
```
sudo apt install librust-openssl-dev
cargo install cargo-generate
git clone https://github.com/bentwire/embassy-rp2040-template.git 
cargo generate --path ./embassy-rp2040-template
```

Once the project has been generated, it is then necessary to update all the of the libraries in 
`cargo.toml` to the most recent version, and also change the channel in `rust-toolchain.toml`.

The `cyw43-firmware` directory needs to be copied into the project directory from the root of the 
embassy directory assuming that the project needs wifi.

`cargo run` to build and flash the target with a debug build. Add `--release` to specify a release build

## MQTT Topics
Messages for each of the topics consists of a single ascii word as described below
- `aircon/temp`: Set aircon temperature (Degrees Celsius)
- `aircon/mode`: Set the aircon mode (`OFF`, `HEAT`, `COOL`, `FAN`)
- `aircon/fan_speed`: Set the aircon fan speed (`1`, `2`, `3`, `4`, `5`, `AUTO`)