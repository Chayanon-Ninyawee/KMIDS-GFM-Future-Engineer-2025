# Changelog

## [0.2.1](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.2.0...v0.2.1) (2025-11-24)


### Bug Fixes

* Fix camera HFOV value ([a07019a](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a07019a5bdc556d9cc0608c75cf16289d5e02873))

## [0.2.0](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.6...v0.2.0) (2025-11-24)


### Features

* Add log_to_video ([6b4a561](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6b4a561095f6ad7b2c5020074f32a4223e50c1c3))
* Add randomizer ([6a730c6](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6a730c68042de092d1243c262ceffc361d50ad19))
* Add unparking ([0725b79](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/0725b79c82d1b0e86e216618f0d4b9f667f311aa))
* Change pico2 code to match to new servo ([16d0cbf](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/16d0cbf4d19d62a0ead5e200f94077c2bd1952a9))
* Change servo to S0009m ([f1f7d38](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/f1f7d38cc288a3885c62ada8f46330f9c00fb5ea))
* Make obstacle_challenge know turn direction when start in the parking lot ([357e9ea](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/357e9ea3be3eadd908b0683bcdb12e1ec62996ae))
* Make the robot Push when it know all the traffic light ([e6ebc04](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/e6ebc04457d14d01e730fa49f284b337adaca4c0))
* Re-slice the FrontCover ([1b9133b](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/1b9133becd7ba550b13069e33e0a839d672cad08))
* Re-slice the Linkages ([c59011c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c59011c5b1d0a00ed8f390af8c047f892694d2d8))


### Bug Fixes

* Check for delta pose error ([5330650](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/5330650c2bac4897c8f94c2da23384ac4ac9cba7))
* Fix camera see parking wall as red and associate that to the traffic light ([31b4a19](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/31b4a197741e015ba6914639b46747f7721ee104))
* Fix CCW parking ([53957f9](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/53957f9bcf40376ac5299809e56d2b4ef2bdafd3))
* Fix logger issue ([4a7a8f7](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4a7a8f780bffa5e4a9756a679c043f2df5bd74e7))
* Fix obstacle_challenge backward pid ([8f61485](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/8f6148572d78b3e689b6c7a4e2ac5a3ec5a92f35))
* Fix obstacle_challenge heading diff calculation ([59ece05](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/59ece05e22e3a91ab1b4b5c5e59ee7faab672b8d))
* Fix obstacle_challenge traffic light detection issue ([822ae8d](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/822ae8d92e3adeb365331d99c43e910c62df9d45))
* Fix open_challenge backward pid ([ad4ab77](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/ad4ab776eec2c03cf4b5d303ee30ff24bbfea90a))
* Fix scan map ([979dbdc](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/979dbdc6edc2410714c98800b33f26083ca9267b))
* Fix traffic light detection issue and CW parking ([bb7ca82](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/bb7ca826213d53da3c94902d144f89272ddb8b65))
* Make build-arm64.sh not fail when failed to pull docker image ([f96658f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/f96658fd7f5915079d58323516af78aefde8a33d))
* Make CW Unpark better ([b077179](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/b0771793c444d3054422d6c8172822dae9b82bd6))
* Make LidarModule read lidar healy error code when timed out 3 times in a row ([3417f34](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/3417f347b1b6e0ee9102a7c6252a963487a01eac))
* Make obstacle_challenge more stable ([74c1493](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/74c1493735c5ca68ac901cb45c6d8ac3cf9dd865))
* Make pushing lap not check for traffic lights ([4b23905](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4b2390551d9ec13fef30b19eed61634a1441ed9f))
* Move lidarData filtering to the correct place ([0bf1d07](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/0bf1d0765e10f61500a1543a9d81b361e62e9460))
* Start camera logging in obstacle_challenge ([59fedce](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/59fedce9350e9707633f02d97b2356babf8372b1))


### Refactor

* Encapsulate open challenge state machine ([05d75f8](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/05d75f8058d8dca053d82276666b21deffb0ff11))
* Refactor obstacle challenge ([e7225c5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/e7225c5e963644ec276be748eca567356a781b5d))
* Refactor open_challenge ([6f74596](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6f745961222d80291d9b3c37eef66697b76ee7e0))
* Rename open challenge mode naming ([9a4abbc](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/9a4abbcc32818041515d9b42266b48b9d9078ebf))


### Miscellaneous Tasks

* Add QOL scripts ([4e07993](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4e0799357786b23a55872766809364f04e7f347f))

## [0.1.6](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.5...v0.1.6) (2025-11-07)


### Documentation

* Add BNO085 draft and fix slicer file and gcode confusion in file section ([4c4d662](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4c4d6622ca5488f35872bca271dd6d6bb5fafedb))
* Add BNO085 specs table and subsection 7 ([b48f3f0](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/b48f3f0ce3d0a5251d49c8dd1d988712401f8835))
* Add brief title to each API README ([90b8f3f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/90b8f3f6eeff70e76419473a00766c04f28a2425))
* Add chassis pic and dimensions ([024cf1c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/024cf1cebe980d25ee212ab159115c0c9b36a7c5))
* Add component pics to building instructions ([5f244fd](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/5f244fd3ad5ea660c3f5b182fe43a25e9fe5f166))
* Add differential gear section to 2.1 ([befae24](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/befae249a6f3cdecdba06aff27e93b0963555701))
* Add M.2 HAT+ to power and sense management draft ([8db13ec](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/8db13ec84c6227d023151df619b59447026bd9a8))
* Add new component pics ([67496df](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/67496dfa1a8866086de51dd00d8b7ec8e6819aa6))
* Add numbering for subsection 7 and clarify 7.1 ([8265d35](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/8265d356ecb567008f7523dddb25e3daa5d804f8))
* Add placeholder section for transistor and motor driver ([5ab1097](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/5ab1097b746f60841f25d0dd16c08a18bf0e161d))
* Add power consumption section draft ([0f9771b](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/0f9771b6c54959a85b4f68066b3027bf678a2a9f))
* Add randomiser files for documentation ([53b00e5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/53b00e56003cd1a266f6c9afccd81c7da809bfc6))
* Add README.md to folders ([612838c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/612838cbd905c44cb6e2583a29700142d096d6ed))
* Add small description paragraphs for processing units ([25f8b91](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/25f8b91e8418673698076f7b50e2d9aee71e404c))
* Align the back to top button to the right ([8b83cf8](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/8b83cf8e0cf27cea0d3bbd13d7cc783e4f0278a1))
* Change all path using backslash to forward slash ([836bda2](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/836bda26ac1b0c51a56dbf3b3f2eea9e70900d9c))
* Change and add robot pics ([22e02fa](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/22e02fa41a1483bbefaf73e9e5a2e1482257688c))
* Change LIDAR pic ([2f54820](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/2f5482084c6b5ae7aac7ab9d56dc9b6b19e51797))
* Change servo from S0004M to S0009M ([a0745da](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a0745da8e1834c1109b09d050d039f681cbb1539))
* Change specs of LIDAR and specify component type ([5eef613](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/5eef613f3ac4cdf3b8f5d265641dcc214ce2cc7a))
* Change wording structure of the steering section ([6fd49a4](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6fd49a4a60d9d3c1625070d38b844ad0ebeafe1d))
* Clean up documentation for typo and wording consistency ([cb82a95](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/cb82a95750bb44a2431bd87e2e41d787b252a021))
* Finalise BNO085 details and Section 9 ([b254871](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/b2548710b1aedf32e92ab364a10535dd5392ebd4))
* Improve readability of the API Documentation landing page ([85d791c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/85d791cdff591413cbcea0868593d4c8bff49ce2))
* List motor driver specs ([2c01a60](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/2c01a6067dbaeaf319dcd8a7db4ddc2290dffa5f))
* Make expandable buttons bigger ([f7bfa6b](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/f7bfa6b1f6524e986e5454d903809090477ee418))
* Move chassis design to mobility management ([4204fe4](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4204fe4ab57e0ca613bb830f41ed3af460f1cdd8))
* Move robot pictures to section 1 ([00a8cd6](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/00a8cd67c2828f2c4f869c0b11a4b702a7ac2fac))
* Move shop link to bill of materials ([4c3d603](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4c3d603715a35961e2e2d8b90497e9bc876958c5))
* Move transistor and motor driver to power section ([3fbc61c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/3fbc61c125a3a6f685bdf8d70c3d7d813805b9e3))
* Move video to section 1 ([7f7bce7](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/7f7bce793345c478f2aed16eff83ffce75bf8f30))
* Reformat building instruction pictures ([bc69ffb](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/bc69ffb76591f80559d4ac9b9a4551323f55ca86))
* Remove lines ([8b62132](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/8b6213213ebd993ee3f5919665772d8b47e73d12))

## [0.1.5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.4...v0.1.5) (2025-10-08)


### Documentation

* Add detailed build settings for each part ([599b20d](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/599b20d4a7740aa936c3569951161c15d32b13f4))
* Add pictures to build instructions 1 and 2 ([e590e7e](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/e590e7ecd25fb36c9dfaec85c475b752ec59f770))
* Add step 1 building instruction pics ([7952c55](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/7952c55b84d7470fdd5a4bac8349f7d009f7c4a5))
* Add step 2 building instruction pics ([261c6cd](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/261c6cdfc5fa5743bc3ba7befe4e394326e637a7))
* Add table of parts in step 0 of building instructions ([839d939](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/839d9392398777d01b835b6fef8596d37430fadb))
* Addjustification for jumper wires in 3.4 ([b5b08ef](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/b5b08ef6e73aaa7c43a7016c7405c39e97cdd89d))
* Change the filename of wiring diagram ([58439b1](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/58439b1e07b8f0ad7eaa64dcfb5516af9547daeb))
* Change wording to ensure consistent wording ([b917324](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/b9173245313bc13a8a5007ee9ff1b6772ba1aeac))
* Format table of contents ([fcd5466](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/fcd54661abd39eb1a1c79a33cf95ec935ebb3be0))
* Remove numbering for STL and Slicer files ([dd9b4ce](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/dd9b4cecc3bd34d5f75783eb74916a2f06c3f8c0))
* Remove redundant table of content sections ([d262324](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/d26232419e334353e47b3c6293f5753fc59b114b))
* Update wiring diagram picture ([af80edf](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/af80edfd91f8ecf0d969b87f492c236a3e709adf))

## [0.1.4](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.3...v0.1.4) (2025-10-05)


### Documentation

* Add docker reference to 7.2 upload instructions ([57b3ee5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/57b3ee5c7915e74bac007470a825d82353379cf4))
* Add links to gcode in step 0 in build instructions ([eb67782](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/eb677827036aec9cdfe113853c7ac18e85d82d91))
* Add more detailed instructions on mounting electronics. ([c64c20a](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c64c20a71c99d5bdd8fd51189494a97deeb27eee))
* Add rough building instructions ([a2df991](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/a2df991b6f364adab4f91c61eda054e3dfc7a239))
* Add settings for step 0 of building instruction ([f12dc55](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/f12dc55ec8e0a838fc21e67b4290b07742654c8e))
* Add slicer files to section 9 ([82b4cd2](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/82b4cd2f8c99fe08654f15a04e1d5c7538108848))
* Add youtube thumbnail, revise step 0 in building instructions ([93344e8](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/93344e87ae38808ff8154b777da821a253b11609))
* Clarify details in step 3 and 4 and add amount specifications to screws ([0d9b6bb](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/0d9b6bb788ec16e06f6baeab1791e7a7567600a5))
* Clarify instructions and components used for step 1 ([0bf8c0f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/0bf8c0f9dc372334abd769c0dc24b9ef68b7311d))
* Clarify instructions and components used for step 2 ([9713236](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/9713236e5ee11602b3c55054a4d9823093cb5b8f))
* Format documentation ([4c93272](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/4c9327200f108f8f3dd8861bbfdbdc44ffce339e))
* Format section 9 points ([d463f2e](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/d463f2efeac19e41bbc6483077a0cb600fb4b02f))
* Make headings in section 9 smaller ([000036b](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/000036be7345818cc1135ae113e69698a026631b))
* Remove part 5 ([bd8a42c](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/bd8a42ca6160a693866ba758f1b30297fd4481f0))
* Remove redundant page lines and add back to top ([2c58b3f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/2c58b3f67c8a28baf3618b977d6b919b9a518984))
* Remove step 4 and 5 from building instructions ([ebf4530](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/ebf4530555f4672ec6d6e4cd039ddac2023610bb))
* Revise building instruction 10 Step 3 ([2d383c5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/2d383c50f395d0e84ce41b486eebb9bc3a44ec5c))
* Revise building instructions step 1 ([881aaaa](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/881aaaa4667308d5a7e3340174082116c6951168))
* Revise step 4 of build instructions ([c98e1ac](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c98e1ac5de7580313c97c63cd48d7e18bc7efcca))
* Specify 9.2 on gcode and settings ([d4a1ae5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/d4a1ae53627eab244209a0736b67ad5bd9f4ec7a))

## [0.1.3](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.2...v0.1.3) (2025-10-05)


### CI/CD

* Fix Build cross-compile image do not cache when it should ([3f68c11](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/3f68c11013e12e49cdb94c5bd05000874614d191))

## [0.1.2](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/compare/v0.1.1...v0.1.2) (2025-10-05)


### Bug Fixes

* Fix obstacle challenge ([6cd3a1e](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6cd3a1eddd2aa1fcef109c3c50347701d5c9c1c7))
* Make combineTraffixLightInfo more robust by using ray tracing algorithm ([84aa4b0](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/84aa4b03220b330a2d8ca8bf174834a2eb74af3c))
* Make getTrafficLightPoints get the deltaPose to properly transpose the trafficLight points according the the data from the IMU ([57aebce](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/57aebce9f154749343c2359b11b67e36225c9172))
* Obstacle challenge: make sure that there can't be outer traffic light in the starting section ([f5242a5](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/f5242a5a48089b11e92739c514d174395cca1585))
* Remove robotDeltaPose from combineTrafficLightInfo so that the trafficLight point does not get transpose twice ([2284e31](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/2284e31a812e0865ba4937bc75da7bd696f85abe))
* Set camera exposure time to be manual instead of auto ([aaeec12](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/aaeec126056323174500699093fdaf2b48711646))


### Miscellaneous Tasks

* Change camera HSV color mask ([adaaf51](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/adaaf517eecb2de688a8c1f458a08094a0e837ec))
* Change camera HSV color mask ([e4631d7](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/e4631d7abc60d150468c65d48974d6024d88ac3a))
* Change lidar filter ([6862b26](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/6862b2691ffeb8af4e35f84e03b4115b26cbf7dc))


### CI/CD

* Restore the correct cache key ([1db539f](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/1db539f4f3351ac3896e31bbabdcd57d9fef8b79))
* Use Docker BuildKit’s built-in GitHub Actions cache ([c04cd26](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/commit/c04cd26f19d3944ed7b921046e18b6183fc745ff))

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
