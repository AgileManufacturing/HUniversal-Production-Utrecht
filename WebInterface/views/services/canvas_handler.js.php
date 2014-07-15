<script type="text/javascript">
	//Disable right mouse button.
	$("#canvas").bind("contextmenu",function(){
       return false;
    });
	
	//For displaying purposes.
	var CANVAS_SIZE_ENLARGEMENT_FACTOR = 5;
	
	var CANVAS_SIZE = 60;
	var SQUARE_SIZE = 5;
	var SQUARE_OFFSET = 2;
	var SQUARE_COLOR = '#c00';
	
	var DISPLAY_SQUARE_SIZE = (CANVAS_SIZE*CANVAS_SIZE_ENLARGEMENT_FACTOR)/(CANVAS_SIZE/SQUARE_SIZE);
	
	
	
	function updateCanvas(){
		DISPLAY_SQUARE_SIZE = (CANVAS_SIZE*CANVAS_SIZE_ENLARGEMENT_FACTOR)/(CANVAS_SIZE/SQUARE_SIZE);
		
		document.getElementById('canvas').width = (CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR);
		document.getElementById('canvas').height = (CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR);
		
		if (typeof update == 'function') { 
			update();
		}
	}
	
	function drawDot(event, clear){
		event = event || window.event;
		
		if (typeof clicked == 'function') { 
			clicked(event,clear);
		}
	}
	
	var mouseDown = false;
	var mouseType = 0;
	$('#canvas').click(function (event){
		if (event.which == 1) drawDot(event, false);
		else drawDot(event, true);
	});
	$('#canvas').mousedown(function (event){
		mouseDown = true;
		mouseType = event.which;
	});
	$(window).mouseup(function(){
		mouseDown = false;
	});
	$('#canvas').mousemove(function (event){
		if (mouseDown){	
			if (mouseType == 1){ //Left mouse button	
				drawDot(event, false);
			}
			else {
				drawDot(event, true);
			}
		}
	});
	
	
</script>