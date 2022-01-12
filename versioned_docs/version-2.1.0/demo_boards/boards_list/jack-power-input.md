# Jack power input board

<div className="cust_sheet" markdown="1">
<p className="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> N/A</p>
<p className="cust_sheet-title" markdown="1"><strong>Type:</strong> N/A</p>
<p className="cust_sheet-title" markdown="1"><strong>Number of service(s):</strong> 0</p>
<p className="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p className="cust_indent" markdown="1"><img height="150" src="/img/jack-power-input-service.png"/></p>
<p className="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p className="cust_indent" markdown="1">
<img height="50" src="/img/sticker-power.png" title="Power"/>
</p>
<p className="cust_sheet-title" markdown="1"><strong>Project source </strong></p>
<a className="github-button" data-size="large" aria-label="Star Luos-io/Luos on GitHub" href="https://github.com/Luos-io/Examples/tree/master/Hardware/wiring_and_power/Jack_power_input" target="_blank">Jack power input</a>
</div>

## Jack power input board functions

The Jack power input board allows to power your Luos Network using a power Jack.

- plug inner diameter : 2 mm
- plug outer diameter : 5.5 mm

See the <a href="https://datasheet.octopart.com/694106301002-W%C3%BCrth-Elektronik-datasheet-111088219.pdf" target="_blank">DC power jack datasheet</a> for more information.

> **Warning:** For your choice of power supply adapter, the max Ampere value should not exceed 5A.

This board is not active, you can't detect it in a network.

You can manage multiple voltage in the same network following Luos power rules defined in [Luos boards general use] or by using a [Power isolator board](./power-isolator.md).

## Power considerations

The Jack power input board can provide 5V to 24V DC.
