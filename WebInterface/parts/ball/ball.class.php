<?php
require_once (__DIR__.'/../part.php');

class Ball extends Part {
	protected $name = 'ball';
	protected $canvas = '';
	protected $targets = array();
	protected $subjects = array('place','placeWithRotation');
}

?>