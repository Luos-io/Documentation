<h1 class="no-break"><a href="#welcome" class="header" id="welcome">Welcome to Luos documentation</a></h1>

<small><a href="https://github.com/Luos-io/Luos" target="blank">Luos revision: {{last_version_luos}}</a></small>

<div class="cust_square_wrapper">
  <div class="cust_square">
    <img class="cust_landing_img" src="./_assets/img/icon-hat_school.png" />
    <div class="cust_landing_title">Get started</div>
    <div class="cust_landing_text">Learn the first step to use Luos.</div>  
  </div>
  <a href="./docs/html/index.html"><div class="cust_square">
    <img class="cust_landing_img" src="./_assets/img/icon-docs2.png" />
    <div class="cust_landing_title">Documentation</div>
    <div class="cust_landing_text">Browse through Documentation to learn more about Luos technology.</div>  
  </div></a>
  <a href="./help/html/index.html"><div class="cust_square">
    <img class="cust_landing_img" src="./_assets/img/icon-help.png" />
    <div class="cust_landing_title">Help</div>
    <div class="cust_landing_text">Consult Help pages to find how to resolve your issue.</div>  
  </div></a>
</div>

## Introduction

We started designing Luos with the conviction that building electronic systems should be made easier than it is today. Most of the time should be spent developing the applications and behaviors instead of on complex and time-and-money-eating technicalities. To give a simple example, adding a new sensor &mdash;for instance, a distance sensor&mdash; to an electronic device in conception should not take more than a few minutes. So you can try, test, and iterate fast on a project to truly design what users want.

**Luos works like a <a href="https://en.wikipedia.org/wiki/Microservices" target="_blank">microservices architecture</a> in the software world and a containerization platform. It encapsulates any software or hardware functions to make them communicate and work with any other encapsulated service, no matter how they were developed, either on bare metal or on top of an embedded OS.**

### You are not familiar with Luos operations? Follow this flowchart:

<div id="service1">

<figure  class="print-break">
  <figcaption></figcaption>
  <ul class="tree">
    <li class="wf_li"><span class="cust_choice"><img src="./_assets/img/logo-luos.png" width="100px"><br /><strong>What do you want to do?</strong></span>
      <ul class="wf_ul">
        <li class="wf_li"><span class="cust_basics"><a name="step2"></a><a name="step3"></a><a href="./pages/overview/general-basics.md">Begin with the <b>basics</b></a></span>
          <ul class="wf_ul">
            <li class="wf_li"><span class="cust_choice"><strong class="cust_number">&#9312;</strong><br />Build a Luos-ready board</span>
              <ul class="wf_ul">
                <li class="wf_li"><span><a href="./pages/embedded/hardware_topics/electronic-design.md">Read about the <b>electronic design rules</b></a></span>
                	<ul class="wf_ul">
                		<li class="wf_li"><span><a href="#step2">Go to 2 <strong>&#8599;</strong></a></span>
                		</li>
                	</ul>
                </li>
              </ul>
            </li>
           <li class="wf_li"><span class="cust_choice"><strong class="cust_number">&#9313;</strong><br />Make <b>drivers</b> and <b>apps</b> for your hardware</span>
              <ul class="wf_ul">
                <li class="wf_li"><span><a href="./pages/embedded/dev-env.md">Choose and configure your <b>development environment</b></a></span>
                	<ul class="wf_ul">
                		<li class="wf_li"><span><a href="./pages/embedded/services.md">Read about <b>services</b> and how they work
</a></span>
                			<ul class="wf_ul">
                				<li class="wf_li"><span><a href="./pages/embedded/services/create-project.md">Create a <b>project</b></a><br /> then <br /><a href="./pages/embedded/services/create-services.md">start creating <b>services</b></a> and <a href="./pages/embedded/services/use-profiles.md"><b>profiles</b></a></span>
                					<ul class="wf_ul">
                						<li class="wf_li"><span>Learn more about the tools and configurations available with Luos:<br /> 
											&#8594; <a href="./pages/embedded/services/od.md"><b>Object dictionary</b></a><br />
											&#8594; <a href="./pages/embedded/services/routing-table.md"><b>Routing table</b></a><br />
											&#8594; <a href="./pages/embedded/services/msg-handling.md"><b>Messages handling</b></a><br />
											&#8594; <a href="./pages/embedded/services/self-healing.md"><b>Self-healing</b></a><br />
											&#8594; <a href="./pages/embedded/services/streaming.md"><b>Streaming</b></a></span>
                							<ul class="wf_ul">
                								<li class="wf_li"><span><a href="./pages/embedded/services/examples.md">Read the Codes Examples</a> and <a href="https://community.luos.io/t/a-new-way-to-design-embedded-app-using-luos-intro/277">follow the bike alarm tutorial</a>
												</span>
													<ul class="wf_ul">
								                		<li class="wf_li"><span><a href="#step3">Go to 3 <strong>&#8599;</strong></a></span>
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
            <li class="wf_li"><span class="cust_choice"><strong class="cust_number">&#9314;</strong><br />Use Luos-ready boards and services</span>
                <ul class="wf_ul">
                	<li class="wf_li"><span><a href="./pages/software/json-api.md#"><img src="./_assets/img/json-logo.png" width="60px"><br />Learn how to use the <b>JSON API</b></a></span>
                		<ul class="wf_ul">
                			<li class="wf_li"><span><a href="./pages/software/pyluos.md"><img src="./_assets/img/python-logo.png" width="60px"><br />Read about <b>Pyluos</b> and how to create behaviors</a></span>
                        <ul class="wf_ul">
                          <li class="wf_li"><span><a href="./pages/software/ros.md"><img src="./_assets/img/ros-logo.png" width="60px"><br />Read about <b>ROS</b> integration and how to use it with Luos</a></span>
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
</figure>
</div>

If you have questions about a specific topic, you can refer or ask it on the <a href="https://community.luos.io" target="_blank">Luos' Forum</a>. And if you have suggestions about this documentation don't hesitate to create pull requests.

Watch this video for additional details:

<iframe class="cust_video" src="https://www.youtube.com/embed/xQe3z0M_FE8?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe><br />


<small>Luos is under <a href="https://github.com/Luos-io/Luos/blob/master/LICENSE" target="_blank">Apache 2.0 license</a>.</small>
