# Changelog

## [0.1.1](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.0...v0.1.1) (2025-10-01)


### CI/CD

* Make cache key base on run number instead of hash since same Dockerfile can have difference cache ([137cafb](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/137cafbd318d167ce982e7589d4ed60951a38f6c))

## [0.1.0](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.0.2...v0.1.0) (2025-10-01)


### Features

* Add build-log_viewer-native.sh ([46b4f8d](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/46b4f8db8afb447ea749bc71274858b2262cde8f))
* Make log_viewer use openChallenge.bin scanMap.bin or obstacleChallenge.bin to get the timestamp of the main loop ([838f1af](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/838f1af3a89877ece414e329f4776f5a47dfcaab))


### Bug Fixes

* Delete the logger after the main loop end to prevent memory leak ([525a663](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/525a663ebacddf0148812b84ca59aa037737956a))
* Faster log_viewer ([a841954](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a841954135d40a6fec8294d855c29df0418455b0))
* Fix build-log_viewer-native.sh so it remove the correct build_native/ not build/ when clean ([a564217](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a564217cd6a02b0c59134d7821d9cf34b436d04c))
* Flush file when logger is deconstructed ([13ec9c6](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/13ec9c6e77aa946786750c625d8c289f405e3f9b))
* Make lidar_processor::getTrafficLightPoints work properly with the new RPLidar S2 ([97daacb](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/97daacb6d2a7ba19c889be637c1c47c20212a4d0))
* Make the bin/ output to the correct directory instead of hardcoded value ([aadc8df](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/aadc8df68b5e16c6ad4058e7040a01db2065dd4b))


### Miscellaneous Tasks

* Change camera color HSV mask ([5665760](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/56657605286af5677d468daf21b5acffc08f3a15))

## [0.0.2](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.0.1...v0.0.2) (2025-09-29)


### Bug Fixes

* Change parking walls angle threshold to ignore shadow parking walls ([c587c09](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c587c095b9e35ca17550dd07082c737e9496604d))

## [0.0.1](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.0.0...v0.0.1) (2025-09-29)


### CI/CD

* Add github workflows to automate release ([7a904f9](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/7a904f92845440cf548a05d511784d56ab686864))
