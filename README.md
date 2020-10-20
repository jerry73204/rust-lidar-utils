# lidar-utils

The crate lets you parse data from Velodyne and Ouster LiDARs. It provides the following functionalities.

- Raw packet encoding and decoding
- Raw data to point cloud conversion
- Ouster LiDAR command API client

## Usage

Add this line to your `Cargo.toml`.

```toml
lidar-utils = "0.7"
```

## Documentation

Please visit [docs.rs](https://docs.rs/lidar-utils/).

## Examples

Check out [Velodyne example](tests/velodyne.rs) and [Ouster example](tests/ouster.rs) in tests.

## License

MIT license. See [LICENSE](LICENSE) file.
