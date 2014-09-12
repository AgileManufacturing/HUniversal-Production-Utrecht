<?php

abstract class Part {
	//Name of the part as stored in the SupplyAgent.
	public function getName(){ return $this->name; }
	public function getCanvas(){ return $this->canvas; }
	public function isServiceTarget($service){ return in_array($service, $this->targets); }
	public function isServiceSubject($service){ return in_array($service, $this->subjects); }	
}

?>