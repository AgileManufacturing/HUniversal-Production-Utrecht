<?php
require_once (__DIR__.'/../part.php');

class Paper extends Part {
	protected $name = 'Paper';
	protected $canvas = 'default';
	protected $targets = array('draw');
	protected $subjects = array();
}

?>