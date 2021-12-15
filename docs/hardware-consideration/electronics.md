---
custom_edit_url: null
---

import { customFields } from "/docusaurus.config.js";
import Tooltip from "/src/components/Tooltip.js";
import Image from '/src/components/Images.js';
import IconExternalLink from '@theme/IconExternalLink';

# Integrating Luos into an electronic board

To create and match with a default reference design, electronic boards must respect some design rules to work in a Luos network properly.

## Electronic design

A Luos-friendly electronic board must contain _at least_ the following elements:

- **1** <a href="https://en.wikipedia.org/wiki/Microcontroller" target="_blank">**MCU<IconExternalLink width="10" />**</a> (microcontroller unit): It hosts, as a node, the Luos firmware along with the different <Tooltip def={customFields.service_def} >services</Tooltip> (drivers and apps).
- **At least two connectors**: They allow to link boards together into a Luos network and link them as a daisy-chain or star mounting. Through PTP pins, nodes know if there is another node connected to the connector. This is used when the user wants to make a topology detection of the system.

## One-wire reference design

<div align="center">
    <Image src="/img/Luos_Network_Interface_OW.svg" darkSrc="/img/Luos_Network_Interface_OW-dark.svg"/>
</div>

Luos' One-wire official connector is <a href="https://octopart.com/df11-4dp-2ds%2852%29-hirose-261749" target="_blank">_DF11-4DP-2DS<IconExternalLink width="10" />_</a>.

## RS485 reference design

<div align="center">
    <Image src="/img/Luos_Network_Interface_485.svg" darkSrc="/img/Luos_Network_Interface_485-dark.svg"/>
</div>

Luos' RS485 official connector is <a href="https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447" target="_blank">_DF11-8DP-2DS<IconExternalLink width="10" />_</a>.

See the default pinout configuration in <a href="https://github.com/Luos-io/LuosHAL" target="_blank">_luos_hal_config.h<IconExternalLink width="10" />_</a> file for chosen the MCU family.
