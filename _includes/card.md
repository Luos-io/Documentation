<div class="sheet" markdown="1">
<p class="sheet-title" markdown="1">**Default Alias:** {{alias}}</p>
<p class="sheet-title" markdown="1">**Type:** {{type}}</p>
{% if {{board}} contains 'Dynamixel' %}
<p class="sheet-title" markdown="1">**Number of module(s):** N</p>
{% else %}
{% assign my_array = {{alias}} | split: ", " %}
<p class="sheet-title" markdown="1">**Number of module(s):** {{ my_array | size }}</p>
{% endif %}
<p class="sheet-title" markdown="1">**Image**</p>
{% if {{board}} contains 'Dynamixel' %}
<p class="indent" markdown="1"><img height="150" src="/assets/img/dxl1-module.png" alt="Dynamixel 1}"><img height="150" src="/assets/img/dxl2-module.png" alt="Dynamixel 2"></p>
{% else %}
<p class="indent" markdown="1"><img height="150" src="/assets/img/{{ board | downcase | replace: ' ', '-' }}-module.png" alt="{{ board | Capitalize }}"></p>
{% endif %}
<p class="sheet-title" markdown="1">**Categories**</p>
<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>
