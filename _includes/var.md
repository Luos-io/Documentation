{% assign act_desc = "Actuation boards are able to act on the physical world." %}
{% assign sen_desc = "Sensor boards are able to measure physical world environment." %}
{% assign com_desc = "Communication boards (also called gates) are able to share your system’s inputs, outputs and configurations outside of your robot, using a JSON API." %}
{% assign cog_desc = "Cognition are boards dedicated to execute your code or host your AI." %}
{% assign int_desc = "These boards are built to interact with the user of the machine." %}
{% assign pow_desc = "Power boards are able to share their input power source into the Robus wire to feed other boards." %}

{% assign node_def = "Hardware element (MCU) running Luos and hosting one or several modules." %}
{% assign module_def = "Software element ran by Luos that can communicate with other modules. Can be driver or app, or both in the same time." %}
{% assign od_def = "Set of typical objects that can be transmitted through Luos messages. Any object can easily be converted in other units." %}


{% assign act_title = "Actuation" %}
{% assign sen_title = "Sensor" %}
{% assign com_title = "Communication" %}
{% assign cog_title = "Cognition" %}
{% assign int_title = "User interface" %}
{% assign pow_title = "Power" %}

{% assign act_img = "{{ "/" | absolute_url }}/assets/img/sticker-actuation.png" %}
{% assign sen_img = "{{ "/" | absolute_url }}/assets/img/sticker-sensor.png" %}
{% assign com_img = "{{ "/" | absolute_url }}/assets/img/sticker-communication.png" %}
{% assign cog_img = "{{ "/" | absolute_url }}/assets/img/sticker-cognition.png" %}
{% assign int_img = "{{ "/" | absolute_url }}/assets/img/sticker-interface.png" %}
{% assign pow_img = "{{ "/" | absolute_url }}/assets/img/sticker-power.png" %}

{% assign last_version_luos = "0.6.0" %}
{% assign last_version_pyluos = "0.1.1" %}
{% assign last_doc_update = "31/12/2019" %}

{% assign table_prop_module = '{:.table_desc}
| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is an unique number given to each module depending on their physical position. The system automatically assign each ID during the detection phase. If you move a module from a microcontroller A to a microcontroller B on a given robot, the ID will change. In the same way, if you change the wiring order of a microcontroler on the network on a given robot, the ID will change too. | Integer<br />e.g. `Id=1` |
| **TYPE** | The type defines the module purpose. A few types are predefined and can be used, or new ones can be created. The module type cannot be changed after module initialization. | String<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the module. It is used to easily identify a module. Each module has a **default alias** who can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we do not want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a module. | String<br />e.g. `alias="gate"` |' %}


