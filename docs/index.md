<h1 className="no-break"><a href="#welcome" className="header" id="welcome">Welcome to Luos documentation</a></h1>

<small><a href="https://github.com/Luos-io/Luos" target="blank">Luos revision: 1.3.0</a></small>

## Introduction

We started designing Luos with the conviction that building electronic systems should be made easier than it is today. Most of the time should be spent on designing the applications and behaviors instead of on complex and time-and-money-eating technicalities. To give a simple example, adding a new sensor &mdash;for instance a distance sensor&mdash; to an electronic device in conception should not take more than a few minutes. So you can try, test and iterate fast on a project to truly design what users want.

**Luos works like a <a href="https://en.wikipedia.org/wiki/Microservices" target="_blank">microservices architecture</a> in the software world, and a containerization platform: it encapsulates any software or hardware function to make it communicate and work with any other encapsulated container, however it was developed, either on bare metal or on top of an embedded OS.**

### You are not familiar with Luos operations? Follow this flowchart:

<div id="container1">

<figure  className="print-break">
  <figcaption></figcaption>
  <ul className="tree">
    <li className="wf_li"><span className="cust_choice"><img src="./_assets/img/logo-luos.png" width="100px" alt="" /><br /><strong>What do you want to do?</strong></span>
      <ul className="wf_ul">
        <li className="wf_li"><span className="cust_basics"><a name="step2"></a><a name="step3"></a><a href="./overview/general-basics.md">Begin with the <b>basics</b></a></span>
          <ul className="wf_ul">
            <li className="wf_li"><span className="cust_choice"><strong className="cust_number">&#9312;</strong><br />Build a Luos-ready board</span>
              <ul className="wf_ul">
                <li className="wf_li"><span><a href="./embedded/hardware_topics/electronic-design.md">Read about the <b>electronic design rules</b></a></span>
                	<ul className="wf_ul">
                		<li className="wf_li"><span><a href="#step2">Go to 2 <strong>&#8599;</strong></a></span>
                		</li>
                	</ul>
                </li>
              </ul>
            </li>
           <li className="wf_li"><span className="cust_choice"><strong className="cust_number">&#9313;</strong><br />Make <b>drivers</b> and <b>apps</b> for your hardware</span>
              <ul className="wf_ul">
                <li className="wf_li"><span><a href="./embedded/dev-env.md">Choose and configure your <b>development environment</b></a></span>
                	<ul className="wf_ul">
                		<li className="wf_li"><span><a href="./embedded/containers.md">Read about <b>containers</b> and how they work
</a></span>
                			<ul className="wf_ul">
                				<li className="wf_li"><span><a href="./embedded/containers/create-project.md">Create a <b>project</b></a><br /> then <br /><a href="./embedded/containers/create-containers.md">start creating <b>containers</b></a> and <a href="./embedded/containers/use-profiles.md"><b>profiles</b></a></span>
                					<ul className="wf_ul">
                						<li className="wf_li"><span>Learn more about the tools and configurations available with Luos:<br /> 
											&#8594; <a href="./embedded/containers/od.md"><b>Object dictionary</b></a><br />
											&#8594; <a href="./embedded/containers/routing-table.md"><b>Routing table</b></a><br />
											&#8594; <a href="./embedded/containers/msg-handling.md"><b>Messages handling</b></a><br />
											&#8594; <a href="./embedded/containers/self-healing.md"><b>Self-healing</b></a><br />
											&#8594; <a href="./embedded/containers/streaming.md"><b>Streaming</b></a></span>
                							<ul className="wf_ul">
                								<li className="wf_li"><span><a href="./embedded/containers/examples.md">Read the Codes Examples</a> and <a href="https://community.luos.io/t/a-new-way-to-design-embedded-app-using-luos-intro/277">follow the bike alarm tutorial</a>
												</span>
													<ul className="wf_ul">
								                		<li className="wf_li"><span><a href="#step3">Go to 3 <strong>&#8599;</strong></a></span>
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
            <li className="wf_li"><span className="cust_choice"><strong className="cust_number">&#9314;</strong><br />Use Luos-ready boards and containers</span>
                <ul className="wf_ul">
                	<li className="wf_li"><span><a href="./software/json-api.md#"><img src="./_assets/img/json-logo.png" width="60px" alt="" /><br />Learn how to use the <b>JSON API</b></a></span>
                		<ul className="wf_ul">
                			<li className="wf_li"><span><a href="./software/pyluos.md"><img src="./_assets/img/python-logo.png" width="60px" alt="" /><br />Read about <b>Pyluos</b> and how to create behaviors</a></span>
                        <ul className="wf_ul">
                          <li className="wf_li"><span><a href="./software/ros.md"><img src="./_assets/img/ros-logo.png" width="60px" alt="" /><br />Read about <b>ROS</b> integration and how to use it with Luos</a></span>
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

<iframe className="cust_video" src="https://www.youtube.com/embed/xQe3z0M_FE8?feature=oembed" frameBorder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowFullScreen></iframe><br />

<small>Luos is under <a href="https://github.com/Luos-io/Luos/blob/master/LICENSE" target="_blank">Apache 2.0 license</a>.</small>
