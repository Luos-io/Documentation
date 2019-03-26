---
layout: default
title:  "Tags"
---

# Modules' categories

In the following lists, the modules are classified by category. Consult the <a href="{{ "/" | absolute_url }}general-use.html"><big>General use</big></a> page to read the description of each category.

{% for tag in site.tags %}
  <h3>{{ tag[0] | capitalize }}</h3>
  <ul>
    {% for post in tag[1] %}
      <li><a href="{{ post.url }}">{{ post.title }}</a></li>
    {% endfor %}
  </ul>
{% endfor %}