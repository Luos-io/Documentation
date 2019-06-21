---
layout: default
title:  "Tags"
---

# Boards' categories

In the following lists, the boards are organized by category. Consult the <a href="{{ "/electronic-use.html" | absolute_url }}"><big>Luos boards general use</big></a> page to read the description of each category.

{% for tag in site.tags %}
  <h3><img height="50" src="/assets/img/sticker-{{ tag[0] }}.png" alt="{{ tag[0] | capitalize }}">&nbsp; {{ tag[0] | capitalize }}</h3>
  <ul>
    {% for post in tag[1] %}
      <li><a href="{{ post.url }}">{{ post.title }}</a></li>
    {% endfor %}
  </ul>
{% endfor %}
