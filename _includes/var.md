{% assign act_desc = "Actuation modules are able to act on the physical world." %}
{% assign sen_desc = "Sensor modules are able to measure physical world environment." %}
{% assign com_desc = "Communication modules (also called gates) are able to share your system’s inputs, outputs and configurations outside of your robot, using a JSON API. <br />You can use these modules to control or program your entire robot with any other device (computer, phone, another robot, etc.)" %}
{% assign cog_desc = "Cognition are modules dedicated to execute your code or host your AI." %}
{% assign int_desc = "These modules are built to interact with the user of the machine." %}
{% assign pow_desc = "Power modules are able to share their input power source into the Robus wire to feed other modules." %}

{% assign act_title = "Actuation" %}
{% assign sen_title = "Sensor" %}
{% assign com_title = "Communication" %}
{% assign cog_title = "Cognition" %}
{% assign int_title = "User interface" %}
{% assign pow_title = "Power" %}

{% assign act_img = "/assets/img/sticker-actuation.png" %}
{% assign sen_img = "/assets/img/sticker-sensor.png" %}
{% assign com_img = "/assets/img/sticker-communication.png" %}
{% assign cog_img = "/assets/img/sticker-cognition.png" %}
{% assign int_img = "/assets/img/sticker-interface.png" %}
{% assign pow_img = "/assets/img/sticker-power.png" %}

{% case module %}
  {% when 'Button' %}
     {% assign type = "Button" %}
  {% when 'Raspberry-Pi' %}
     {% assign type = "-" %}
  {% when 'Wifi-BLE' %}
     {% assign type = "-" %}
  {% when 'Controlled-motor' %}
     {% assign type = "ControlledMotor" %}
  {% when 'DC-motor' %}
     {% assign type = "DCMotor" %}
  {% when 'Distance' %}
     {% assign type = "Distance" %}
  {% when 'Dynamixel' %}
     {% assign type = "DynamixelMotor" %}
  {% when 'GPIO' %}
     {% assign type = "GPIO" %}
  {% when 'IMU' %}
     {% assign type = "IMU" %}
  {% when 'Light' %}
     {% assign type = "LightSensor" %}
  {% when 'Potentiometer' %}
     {% assign type = "Potentiometer" %}
  {% when 'Power-switch' %}
     {% assign type = "PowerSwitch" %}
  {% when 'RGB-LED' %}
     {% assign type = "LED" %}
  {% when 'Servomotor' %}
     {% assign type = "Servo" %}
  {% when 'Stepper' %}
     {% assign type = "Stepper" %}
  {% when 'USB' %}
     {% assign type = "-" %}
 
  {% else %}
     {% assign type = "-" %}
{% endcase %}


{% assign last_version = "0.3.2" %}
