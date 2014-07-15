<script src="websocket/service_receiver.js"></script>
<script>
	var productStepHandler;
	
	
    function loadService(){
        var service_selector = document.getElementById('service_selector');
        var service = service_selector.options[service_selector.selectedIndex].value;
		
		
		$.when($.ajax({
		  url: './views/services/' + service + "/" + service + '.php',
		  type: "POST",
		  data: { }
		})).then(function( msg ) {
			if(service=="placeWithRotation"){
				productStepHandler = new ProductStepHandler("place");
			}
			else{
				productStepHandler = new ProductStepHandler(service);
			}
			document.getElementById("service").innerHTML = msg;
            $("#service").find("script").each(function(i) {
                eval($(this).text());
            });
		});
    }
    
    function sendProductSteps(json){
		var numProducts = document.getElementById('number_of_products').value;
		
		for (var i=0;i<numProducts;i++){
			sendWebsocketMessage('{"receiver":"webSocketServer", "subject":"create_product", ' + json + ' }');
		}
    }
	
	function loadPart(part_list, part_list_extended){
		console.log('loading: ' + part_list_extended);
        var part = part_list.options[part_list.selectedIndex].value;
			
		var targeti = new Array();
		targeti.identifier = part;
		productStepHandler.setProperty('target',targeti);
		
		
		$.when($.ajax({
		  url: './parts/' + part + "/" + part + '.inc.php',
		  type: "POST",
		  data: { }
		})).then(function( msg ) {
			document.getElementById(part_list_extended).innerHTML = msg;
			var extended = '#' + part_list_extended;
            $(extended).find("script").each(function(i) {
                eval($(this).text());
            });
		});
		
		$.when($.ajax({
		  url: './canvases/CanvasLoader.inc.php',
		  type: "POST",
		  data: { part:part, data:'update' }
		})).then(function( msg ) {
		console.log("Message: "+msg);
            update = Function(msg);
			$.when($.ajax({
			  url: './canvases/CanvasLoader.inc.php',
			  type: "POST",
			  data: { part:part, data:'clicked' }
			})).then(function( msg ) {
				clicked = Function('event','clear',msg);
				updateCanvas();
			});
		});
		
    }
</script>


<form class="form-horizontal row" role="form">
    <div class="col-sm-12">
        <div class="form-group" style="display:none;">
    		<label for="service_selector" class="col-sm-3 control-label" style="text-align:left">
            	Number of products:
            </label>
			<div class="col-sm-9">
      			<div class="input-group">
  					<input id="number_of_products" type="number" class="form-control" value="1">
  					<span class="input-group-addon">pieces</span>
				</div>
    		</div>
  		</div>
        <div class="form-group">
            <label for="service_selector" class="col-sm-3 control-label" style="text-align:left">
                Select your service:
            </label>
            <div class="col-sm-9">
                <select id="service_selector" onchange="loadService()" class="form-control">
                <?php
                    foreach (new DirectoryIterator('./views/services') as $fileInfo) {
                        if($fileInfo->isDot() || !$fileInfo->isDir()) continue;
                        ?><option><?=$fileInfo->getFilename()?></option><?php
                    }
                ?>
                </select>
            </div>
        </div>
    </div>
</form>


 
<script> loadService(); </script>

<!-- Container where the service will be loaded -->
<div id="service" class="row">
</div>

<div id="canvas_container" class="col-sm-6">
    <canvas id="canvas" width="300" height="300"></canvas><br />
</div>

<div class="send_container">
	<button type="button" id="send" class="send_btn btn btn-warning ">Request product(s)</button>
</div>

<?php include(__DIR__.'/canvas_handler.js.php'); ?>

<script>
	$('#send').click(function (){
		var json = '"productSteps":[';
		var jsonPSH = productStepHandler.toJSON();
		json += jsonPSH.substring(1, jsonPSH.length - 1);
		json += ']';
		sendProductSteps(json);
	});
</script>