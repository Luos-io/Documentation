# Welcome to Luos documentation

## Introduction

We started designing Luos with the conviction that building electronic systems should be made easier than it is today. Most of the time should be spent on designing the applications and behaviors instead of on complex and time-and-money-eating technicalities. To give a simple example, adding a new sensor &mdash;for instance a distance sensor&mdash; to an electronic device in conception should not take more than a few minutes. So you can try, test and iterate fast on a project to truly design what users want.

**Luos works like <a href="https://en.wikipedia.org/wiki/Microservices" target="_blank">microservices architecture</a> in the software world, and a <a href="https://en.wikipedia.org/wiki/Distributed_operating_system" target="_blank">distributed operating systems</a>: it encapsulates any software or hardware function to make it communicate and work with any other encapsulated module, however it was developed, either on bare metal or on top of an embedded OS.**

### You are not familiar with Luos operations? Follow this flowchart:

<div id="container1">

<figure>
  <figcaption></figcaption>
  <ul class="tree">
    <li class="wf_li"><span class="cust_choice"><strong>What do you want to do?</strong></span>
      <ul class="wf_ul">
        <li class="wf_li"><span class="cust_basics"><a name="step2"></a><a name="step3"></a><a href="/pages/overview/general-basics.md">Begin with the basics</a></span>
          <ul class="wf_ul">
            <li class="wf_li"><span class="cust_choice"><strong class="cust_number">❶</strong><br />Build a Luos-ready board</span>
              <ul class="wf_ul">
                <li class="wf_li"><span><a href="/pages/low/electronic-design.md">Read about the electronic design rules</a></span>
                	<ul class="wf_ul">
                		<li class="wf_li"><span><a href="#step2">Go to 2 <strong>↗</strong></a></span>
                		</li>
                	</ul>
                </li>
              </ul>
            </li>
           <li class="wf_li"><span class="cust_choice"><strong class="cust_number">❷</strong><br />Make drivers and apps for your hardware</span>
              <ul class="wf_ul">
                <li class="wf_li"><span><a href="/pages/low/dev-env.md">Choose and configure your development environment</a></span>
                	<ul class="wf_ul">
                		<li class="wf_li"><span><a href="/pages/low/modules.md">Read about modules and how they work
</a></span>
                			<ul class="wf_ul">
                				<li class="wf_li"><span><a href="/pages/low/modules/create-project.md">Create a project</a> and <a href="/pages/low/modules/create-modules.md">and start creating modules</a></span>
                					<ul class="wf_ul">
                						<li class="wf_li"><span>Learn more about the tools and configurations available with Luos:<br /> 
											➜ <a href="/pages/low/modules/od.md">Object dictionary</a><br />
											➜ <a href="/pages/low/modules/routing-table.md">Routing table</a><br />
											➜ <a href="/pages/low/modules/msg-handling.md">Messages handling</a><br />
											➜ <a href="/pages/low/modules/streaming.md">Streaming</a><br />
											➜ <a href="/pages/low/modules/rt-config.md">Real-time configuration</a></span>
                							<ul class="wf_ul">
                								<li class="wf_li"><span><a href="/pages/low/modules/examples.md">Read the Codes Examples</a> and <a href="#">follow the bike alarm tutorial (soon)</a>
												</span>
													<ul class="wf_ul">
								                		<li class="wf_li"><span><a href="#step3">Go to 3 <strong>↗</strong></a></span>
								                		</li>
								                	</ul>
                								</li>
                							</ul>
                						</li>
                					</ul>
                				</li>
                			</ul>
                		</li>
                	</ul>
                </li>
              </ul>
            </li>
            <li class="wf_li"><span class="cust_choice"><strong class="cust_number">❸</strong><br />Use Luos-ready boards and modules</span>
                <ul class="wf_ul">
                	<li class="wf_li"><span><a href="/pages/high/json-api.md#">Learn how to use the JSON API</a></span>
                		<ul class="wf_ul">
                			<li class="wf_li"><span><a href="/pages/high/pyluos.md">Read about Pyluos and how to create behaviors</a></span>
                			</li>
                		</ul>
                	</li>
                </ul>
            </li>
          </ul>
        </li>
      </ul>
    </li>
  </ul>
</figure>
</div>

If you have questions about a specific topic, you can refer or ask it on the <a href="https://community.luos.io" target="_blank">Luos' Forum</a>. And if you have suggestions about this documentation don't hesitate to create pull requests.

Watch this video for additional details:

<iframe class="cust_video" src="https://www.youtube.com/embed/xQe3z0M_FE8?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe><br />


<small>Luos is free for non-commercial use. It's protected by a license that <a href="https://github.com/Luos-io/Luos/blob/master/LICENSE.md" target="_blank">you can consult here</a>.</small>

<div class="cust_edit_page"><a href="https://{{gh_path}}/index.md">Edit this page</a></div>
