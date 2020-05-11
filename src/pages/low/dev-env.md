# Setup your development environment
Before starting developing with Luos, you need to have an operational development environment.
At Luos, we use PlatformIO to share all our examples and to make our lib integration easy, but of course you can use your favorite IDE and integrate our libs by yourself.

## Setup a Luos PlatformIO project
<a href="https://platformio.org/" target="_blank">PlatformIO</a> is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. You can put it as a plug-in in a lot of <a href="https://docs.platformio.org/en/latest/integration/ide/index.html#desktop-ide" target="_blank">different editors</a>.

### Getting Started
 1. Install Platform IO on VSCode by following the instructions on <a href="https://platformio.org/platformio-ide" target="_blank">this page</a>.
 2. Create a new projet on PlatformIO
 3. Add Luos as dependancy and select HAL on your `platformio.ini` file:

```Json
lib_deps = Luos
build_flags = -D HAL=hal_name
```

> *Note:* More information about how Luos libs are managed into PlatformIO is available by <a href="https://community.luos.io/t/how-to-link-luos-and-robus-to-platformio/244" target="\_blank">following this post on our forum</a>.

### Project examples
Luos shares a lot of <a href="https://github.com/Luos-io/Luos/tree/master/examples/" target="_blank">code examples</a>, feel free to use and modify them as you want.

### Prototyping boards
Luos created [a sets of boards](/pages/prototyping_boards/boards-list.md) allowing to easily prototype a device.

## General integration consideration

Luos works as a library. Our technology is constituted of 2 libs:

 - <a href="https://github.com/Luos-io/Robus" target="_blank"><span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span></a>: This library is the communication way used by Luos.
 - <a href="https://github.com/Luos-io/Luos/tree/master/luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work on your environment, you have to:

 - Include both lib folders (Robus and Luos) in your project compilation,
 - Select the good hal folder to use, depending on you hardware,
 - Include Luos.h on your source file.

> *Note:* Robus and Luos repositories are pre-releases with limited access. Complete sources will be available soon.

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/dev-env.md">Edit this page</a></div>
