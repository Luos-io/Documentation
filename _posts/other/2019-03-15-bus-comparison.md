---
layout: post
title: "Bus comparison"
categories: 4_others
desc: Comparison of different buses and Luos.
order: 0
wip: 1
---
{% include var.md %}

<div class="wip_img"></div>
<blockquote class="warning"><strong>Work in progress</strong><br /><br />We are working on this page...</blockquote><br />

### The following table compares several technical characteristics between Luos and other communication buses.

<table class="bus">
	<tbody>
		<tr class="head">
			<td colspan="2">&nbsp;</td>
			<td class="luos">
				<strong>Luos</strong>
			</td>
			<td>
				<strong>CANopen</strong>
			</td>
			<td>
				<strong>CAN</strong>
			</td>
			<td>
				<strong>TTCAN</strong>
			</td>
			<td>
				<strong>LIN</strong>
			</td>
			<td>
				<strong>Ethercat</strong>
			</td>
			<td>
				<strong>KNX</strong>
			</td>
		</tr>
		<tr>
			<td class="head" rowspan="8">
				<strong>Bus</strong>
			</td>
			<td class="head">
				<strong>Max Speed</strong>
			</td>
			<td class="luos">
				10 Mbit/s
			</td>
			<td colspan="2">
				1 Mbit/S
			</td>
			<td>
				10 Mbit/s
			</td>
			<td>
				40 kbit/s
			</td>
			<td>
				200 Mbit/s
			</td>
			<td>
				2.4 Kbit/s
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Latency</strong>
			</td>
			<td class="luos">
				Low
			</td>
			<td colspan="2">
				Medium
			</td>
			<td>
				Medium
			</td>
			<td>
				Low
			</td>
			<td>
				High
			</td>
			<td>
				High
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Cost</strong>
			</td>
			<td class="luos">
				$
			</td>
			<td colspan="2">
				$$
			</td>
			<td>
				$$$
			</td>
			<td>
				$
			</td>
			<td>
				$$$
			</td>
			<td>
				$$
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Wire</strong>
			</td>
			<td class="luos">
				3 to 4
			</td>
			<td colspan="2">
				2
			</td>
			<td>
				2
			</td>
			<td>
				1
			</td>
			<td>
				2
			</td>
			<td>
				2
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Messaging</strong>
			</td>
			<td class="luos">
				Event &amp; TimeTriggered Messages
			</td>
			<td colspan="2">
				Event Triggered Messages
			</td>
			<td>
				Event &amp; Time Triggered Messages
			</td>
			<td>
				Deterministic, Static Message Scheduling
			</td>
			<td>
				Daisy chain, Deterministic, Static Message Scheduling
			</td>
			<td>
				Event Triggered Messages
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Network Synchronization</strong>
			</td>
			<td class="luos">
				Priority based arbitration
			</td>
			<td colspan="2">
				Priority based arbitration (CSMA)
			</td>
			<td>
				Global reference time (TDMA)
			</td>
			<td>
				Global reference time (TDMA)
			</td>
			<td>
				Ring based
			</td>
			<td>
				Priority based arbitration (CSMA)
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Node Control</strong>
			</td>
			<td class="luos">
				Multi-master
			</td>
			<td colspan="2">
				Master/Slave
			</td>
			<td>
				Multi-master
			</td>
			<td>
				Master/Slave
			</td>
			<td>
				Master/Slave
			</td>
			<td>
				Multi-master
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Detection</strong>
			</td>
			<td class="luos">
				Topology based or Manual
			</td>
			<td>
				Manual or software
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				Topology based
			</td>
			<td>
				Topology based or Manual
			</td>
		</tr>
		<tr>
			<td class="head" rowspan="3">
				<strong>API</strong>
			</td>
			<td class="head">
				<strong>Object dictionary</strong>
			</td>
			<td class="luos">
				Yes
			</td>
			<td>
				Yes
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				Yes
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Virtual uni-microcontroller</strong>
			</td>
			<td class="luos">
				Yes
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
		</tr>
		<tr>
			<td class="head">
				<strong>Features/App packaging</strong>
			</td>
			<td class="luos">
				Yes
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
			<td>
				No
			</td>
		</tr>
	</tbody>
</table>