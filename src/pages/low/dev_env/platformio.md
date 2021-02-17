# Setup a Luos PlatformIO project

<a href="https://platformio.org/" target="_blank">PlatformIO</a> is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. You can put it as a plug-in in a lot of <a href="https://docs.platformio.org/en/latest/integration/ide/index.html#desktop-ide" target="_blank">different editors</a>.

### Getting Started
 1. Install Platform IO on VSCode by following the instructions on <a href="https://platformio.org/platformio-ide" target="_blank">this page</a>.
 2. Create a new projet on PlatformIO
 3. Add Luos as dependancy and select HAL on your `platformio.ini` file:

```Json
lib_deps = Luos
board = <board name>
```
Replace `<board name>` with the name of the board you're using, eg. `board = l0` for the L0 board.

> *Note:* More information about how Luos libs are managed into PlatformIO is available by <a href="https://community.luos.io/t/how-to-link-luos-with-platformio/303" target="\_blank">following this post on our forum</a>.

### Project examples
Luos shares a lot of <a href="https://github.com/Luos-io/Examples" target="_blank">code examples</a>, feel free to use and modify them as you want.

### Demonstration boards
Luos created [a sets of boards](../../demo_boards/boards-list.md) allowing to easily test the technology.
