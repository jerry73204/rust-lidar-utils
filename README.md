# lidar-buffer

The crate aims to encode and decode packets from lidar devices. Currently support Velodyne's and Ouster's packet format.

The repository is under development, and is not available on crates.io until it's stablized.

## Usage

Put this line to include this crate to your project.

```toml
lidar-buffer = { git = "https://github.com/jerry73204/rusts-lidar-buffer", branch = "master" }
```

## Documentation

Clone this repo and run `cargo doc` to compile the documents. The online reference will be available as soon as it is published to docs.io.

## Examples

Check out [Velodyne example](tests/velodyne.rs) and [Ouster example](tests/ouster.rs) in tests.

## License

MIT
