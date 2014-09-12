<?php
require_once (__DIR__.'/../part.php');

class AngledCrate extends Part {
	protected $name = 'angledcrate';
	protected $canvas = 'circleWithOffsetAndRotation';
	protected $targets = array('placeWithRotation');
	protected $subjects = array();
}

?>