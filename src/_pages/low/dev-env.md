# Setup your development environment
Before starting developing with Luos, you need to have an operational development environment.
At Luos, we use Platformio to share all our examples and to make our lib integration easy, but of course you can use your favorite IDE and integrate our libs by yourself.

## Setup a Luos PlatformIO project
<a href="https://platformio.org/" target="_blank">PlatformIO</a> is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. You can put it as a plug-in in a lot of <a href="https://docs.platformio.org/en/latest/integration/ide/index.html#desktop-ide" target="_blank">different editors</a>.

### Getting Started
 1. <a href="https://platformio.org/platformio-ide" target="_blank">Install platformio</a>
 2. Create a new projet on platformio
 3. Add Luos as dependancy and select HAL on your platformio.ini file:

   ```Json
    lib_deps = Luos
    build_flags = -D HAL=hal_name
    ```

> *Note:* More information about how Luos libs are managed into platformio is available by <a href="https://community.luos.io/t/how-to-link-luos-and-robus-to-platformio/244" target="_blank">following this post on our forum</a>.

### Project examples
Luos shares a lot of <a href="https://github.com/Luos-io/Modules" target="_blank">codes examples</a>, feel free to use and modify them as you want.

### Development boards
Luos created [a sets of boards](/_pages/prototyping_boards/boards-list.md) allowing to easily prototype a device.

Luos also shares some [development boards](/_pages/development_boards/dev-board-list.md) allowing you to create from scratch and debug easily.

## General integration consideration

Luos work as a library. Our technology is constituted of 2 libs :

 - <a href="https://github.com/Luos-io/Pre_robus" target="_blank">Robus</a>: This lib is the communication way used by Luos.
 - <a href="https://github.com/Luos-io/Pre_luos" target="_blank">Luos</a>: This is the main lib you will have to deal with.

To make it work on your environment, you have to:

 - Include both lib folders (Robus and Luos) in your project compilation
 - Select the good hal folder to use, depending on you hardware
 - Include Luos.h on your source file.

> *Note:* Robus and Luos repository are pre-release with limited access. Complete sources will be available soon.

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/dev-env.md">Edit this page</a></div>
