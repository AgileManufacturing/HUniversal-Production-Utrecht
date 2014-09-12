<?php

if (isset($_REQUEST['part']) && isset($_REQUEST['data'])){
	$part = $_REQUEST['part'];
	$data = $_REQUEST['data'];
	
	require_once(__DIR__.'/../parts/'.$part.'/'.$part.'.class.php');
	$classname = ucwords(strtolower($part));
	$partObject = new $classname();
	
	$canvas = $partObject->getCanvas();
	require_once($canvas.'.php');
	$classname = ucwords(strtolower($canvas)).'Canvas';
	$c = new $classname();
	
	if ($data == 'update'){
		echo ($c->updateCanvas());
	}
	if ($data == 'clicked') {
		echo ($c->canvasClicked());
	}
}

?>