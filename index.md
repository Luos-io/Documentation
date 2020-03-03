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
