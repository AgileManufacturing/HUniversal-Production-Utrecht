<?php
require_once (__DIR__.'/../part.php');

class Crate extends Part {
	protected $name = 'crate';
	protected $canvas = 'circleWithOffset';
	protected $targets = array('place');
	protected $subjects = array();
}

?>