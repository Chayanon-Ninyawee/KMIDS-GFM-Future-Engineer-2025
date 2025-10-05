# Changelog

## [0.1.2](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.1...v0.1.2) (2025-10-05)


### Bug Fixes

* Fix obstacle challenge ([daf877f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/daf877fc35a3554a3ac8e39464d697e27b0647b5))
* Make combineTraffixLightInfo more robust by using ray tracing algorithm ([a5b6220](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a5b62201dece7c86bfa3781eb1a6f19b6949d925))
* Make getTrafficLightPoints get the deltaPose to properly transpose the trafficLight points according the the data from the IMU ([681fd48](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/681fd489720bae581f1948604aa3e905895b8a24))
* Obstacle challenge: make sure that there can't be outer traffic light in the starting section ([ebaff0e](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/ebaff0e54537a00fd8836dc5e37dcb191785278e))
* Remove robotDeltaPose from combineTrafficLightInfo so that the trafficLight point does not get transpose twice ([94fbc11](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/94fbc118835ccba080265aea58c5bd27d67bd303))
* Set camera exposure time to be manual instead of auto ([c7daaa9](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c7daaa93bbb862df6eeea5d57471192d37e5d464))


### Miscellaneous Tasks

* Change camera HSV color mask ([39804b1](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/39804b1c8584f7d2e9560064b61d768eb22d26fc))
* Change camera HSV color mask ([bc61d6a](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/bc61d6ad0c9a64b06d1642e08a05be8e44ea265d))
* Change lidar filter ([62421bb](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/62421bbe563540190fba35ee31bc48f65394da7e))


### CI/CD

* Restore the correct cache key ([7b4eb3e](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/7b4eb3e2a9ad44953c203f37bf7be08d68f1bb65))
* Use Docker BuildKit’s built-in GitHub Actions cache ([db1dbc8](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/db1dbc84c9c39466de8faddac629d14a7120c5bf))

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
