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

{% assign last_version_fw = "0.5.1" %}
{% assign last_version_pyluos = "0.1.1" %}
