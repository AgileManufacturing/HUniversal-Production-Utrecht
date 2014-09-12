<?php
	require_once(__DIR__.'/../Service.class.php');
	
	$draw = new Service('placeWithRotation');
	$draw->addInput(Language::getText('GAP_SIZE'), INPUT_NUMBER, array('id'=>'gap_size', 'value'=>'10'), '', 'mm');
	$draw->addInput(Language::getText('GAP_SPACING'), INPUT_NUMBER, array('id'=>'gap_spacing', 'value'=>'1'), '', 'mm');
	
	echo $draw->buildService();
?>
<script type="text/javascript">
	SQUARE_SIZE = (document.getElementById('gap_size').value); updateCanvas();
	SQUARE_OFFSET = (document.getElementById('gap_spacing').value); updateCanvas();

	$("#gap_size").keyup(function() {SQUARE_SIZE = (document.getElementById('gap_size').value); updateCanvas();});
	$("#gap_size").change(function() {SQUARE_SIZE = (document.getElementById('gap_size').value); updateCanvas();});
	$("#gap_spacing").keyup(function() {SQUARE_OFFSET = (document.getElementById('gap_spacing').value); updateCanvas();});
	$("#gap_spacing").change(function() {SQUARE_OFFSET = (document.getElementById('gap_spacing').value); updateCanvas();});
</script>