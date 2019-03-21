---
layout: default
title:  "Tags"
date:   2019-03-20 16:43:00 +0100
---

# Modules' categories

In the following lists, the modules are classified by category.

{% for tag in site.tags %}
  <h3>{{ tag[0] }}</h3>
  <ul>
    {% for post in tag[1] %}
      <li><a href="{{ post.url }}">{{ post.title }}</a></li>
    {% endfor %}
  </ul>
{% endfor %}