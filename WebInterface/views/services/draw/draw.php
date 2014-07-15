<?php
	require_once(__DIR__.'/../Service.class.php');
	
	$draw = new Service('draw');
	$draw->addInput(Language::getText('DOT_SIZE'), INPUT_NUMBER, array('id'=>'dot_size', 'step'=>'0.1', 'value'=>'5'), '', 'mm');
	$draw->addInput(Language::getText('COLOR'), INPUT_SELECT, array('id'=>'color', 'step'=>'0.1', 'value'=>'5'), '', '', array('red'));
	
	echo $draw->buildService();
?>

<script type="text/javascript">
	SQUARE_SIZE = (document.getElementById('dot_size').value); updateCanvas();
	
	$("#dot_size").keyup(function() {SQUARE_SIZE = (document.getElementById('dot_size').value); updateCanvas();});
	$("#dot_size").change(function() {SQUARE_SIZE = (document.getElementById('dot_size').value); updateCanvas();});
</script>