<div class="form-group">
    <label for="service_selector" class="col-sm-6 control-label" style="text-align:left">
        Paper size:
    </label>
    <div class="col-sm-6">
        <div class="input-group">
            <input id="paper_size" type="number" class="form-control" value="100">
            <span class="input-group-addon">mm</span>
        </div>
    </div>
</div>
<script>
	CANVAS_SIZE = 100;
	$("#paper_size").keyup(function() {CANVAS_SIZE = (document.getElementById('paper_size').value); updateCanvas();});
	$("#paper_size").change(function() {CANVAS_SIZE = (document.getElementById('paper_size').value); updateCanvas();});
</script>