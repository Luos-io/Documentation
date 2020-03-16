---
layout: default
---
{% include var.md %}

# Welcome to Luos documentation

<span style="font-size: 20px;">Discover how Luos works and how to use it.</span>

<hr class="hr_top">

{% assign cat_order = site.categories | sort: categories %}
{%- for cat in cat_order -%}
{%- assign category = cat | first -%}
{% if category contains "modules_list" or category contains "boards_list" %}{% continue %}{% endif %}
## {{ cat | first | slice: 2, 30 | replace:"_", " " | capitalize }}
{% if category == "prototyping_boards" %}
Luos provides [prototyping boards]("https://www.generationrobots.com/fr/256_luos") to start understanding and using the technology.
{% endif %}
<ul>
{% assign sorted = cat.last | sort: "order" %}
{% for post in sorted %}
<li><a class="{% if post.wip == 1 %}wip_txt{% endif %}" href="{{ post.url }}">{{ post.title }}</a> – {{ post.desc }}</li>
{% endfor %}
{%- assign test_cat = cat | first -%}
{%- if test_cat contains 'modules' -%}
	<li class="ul_tree"><span class="branch_closed">Modules list</span>
		<ul class="closed">
			{%- assign sorted_post = site.posts | sort: 'title' -%}
			{%- for post in sorted_post -%}
			{%- if post.url contains 'modules_list' -%}
			<li class="branch_single"><a href="{{ post.url }}">{{ post.title }}</a></li>
			{%- endif -%}
			{%- endfor -%}
		</ul>
	</li>
{%- endif -%}
{%- if test_cat contains 'prototyping_boards' -%}
	<li class="ul_tree"><span class="branch_closed">Boards list</span>
		<ul class="closed">
			{%- assign sorted_post = site.posts | sort: 'title' -%}
			{%- for post in sorted_post -%}
			{%- if post.url contains 'boards_list' -%}
			<li class="branch_single"><a href="{{ post.url }}">{{ post.title }}</a></li>
			{%- endif -%}
			{%- endfor -%}
		</ul>
	</li>
{%- endif -%}
</ul>
{%- endfor -%}

If the Luos technology is something new for you, you should start by reading these pages allowing you to understand basic concepts.

**Today, Luos is exactly like a [<span class="tooltip">microservices<span class="tooltiptext">{{ microservices_def }}</span></span> architecture](https://en.wikipedia.org/wiki/Microservices){:target="_blank"}: it encapsulates any software or hardware function to make it communicate and work with any other encapsulated module, however it was developed.**

We have imagined Luos around three key concepts:
 * **Luos Core:** a tiny library that can be added to the MCU of an embedded board to make it seamlessly communicate with other boards. When conceiving electronic devices, you should be able to compose it from multiple functional parts and not think about communication issues between these parts.
 * **Luos Driver:** a hardware abstraction to make the integration of a sensor, an effector or a software function as easy at it should be &mdash;that is, with one line of code! Drivers must use standardized APIs to let you switch from one type of sensors to another without breaking anything. For example, all types of distance sensors should return a distance in meters using a same function `get_distance()`.
 * **Robus:** A communication bus dedicated to embedded system communication, to make all the above natural and simple. 

* [General use]({{ "/general-use.html" | absolute_url }}) – What is Luos technology and how to use it.
* [Quick start]({{ "/quick-start.html" | absolute_url }}) – A short tutorial to help getting started with modules.
* [Pyluos]({{ "/pyluos.html" | absolute_url }}) – An explanation of the pyluos library we use in our examples.
* [Luos boards general use]({{ "/electronic-use.html" | absolute_url }}) – Rules to understand and use Luos electronic boards.
* [Firmware update]({{ "/update-module-firmware.html" | absolute_url }}) – How to update your modules firmware.

The pages displayed on the left give inputs about the module functionalities and how to use them on Luos boards.

<span style="font-size: 20px;">Discover how Luos works and how to use it:</span>

{% assign cat_order = site.categories | sort: categories %}
{%- for cat in cat_order -%}
{%- assign category = cat | first -%}
{% if category contains "modules_list" or category contains "boards_list" %}{% continue %}{% endif %}
## {{ cat | first | slice: 2, 30 | replace:"_", " " | capitalize }}
{% if category == "3_prototyping_boards" %}
Luos provides [prototyping boards](https://www.generationrobots.com/fr/256_luos){:target="_blank"} to start understanding and using the technology.
{% endif %}
<ul>
{% assign sorted = cat.last | sort: "order" %}
{% for post in sorted %}
<li><a class="{% if post.wip == 1 %}wip_txt{% endif %}" href="{{ post.url }}">{{ post.title }}</a> – {{ post.desc }}</li>
{% endfor %}
{%- assign test_cat = cat | first -%}
{%- if test_cat contains 'modules' -%}
	<li class="ul_tree"><span class="branch_closed">Modules list</span>
		<ul class="closed">
			{%- assign sorted_post = site.posts | sort: 'title' -%}
			{%- for post in sorted_post -%}
			{%- if post.url contains 'modules_list' -%}
			<li class="branch_single"><a href="{{ post.url }}">{{ post.title }}</a></li>
			{%- endif -%}
			{%- endfor -%}
		</ul>
	</li>
{%- endif -%}
{%- if test_cat contains 'prototyping_boards' -%}
	<li class="ul_tree"><span class="branch_closed">Boards list</span>
		<ul class="closed">
			{%- assign sorted_post = site.posts | sort: 'title' -%}
			{%- for post in sorted_post -%}
			{%- if post.url contains 'boards_list' -%}
			<li class="branch_single"><a href="{{ post.url }}">{{ post.title }}</a></li>
			{%- endif -%}
			{%- endfor -%}
		</ul>
	</li>
{%- endif -%}
</ul>
{%- endfor -%}

{% for cat in site.categories %}
{% assign category = cat | first %}
{% if category == "others" %}
## {{ cat | first | replace:"_", " " | capitalize }}
<ul>
{% assign sorted = cat.last | sort: "order" %}
{% for post in sorted %}
<li><a class="{% if post.wip == 1 %}wip_txt{% endif %}" href="{{ post.url }}">{{ post.title }}</a> – {{ post.desc }}</li>
{% endfor %}
{% endif %}
{% endfor %}
