<div class="form-group">
    <label for="service_selector" class="col-sm-6 control-label" style="text-align:left">
        Gap amount:
    </label>
    <div class="col-sm-6">
        <div class="input-group">
            <input id="paper_size" type="number" class="form-control" value="4">
            <span class="input-group-addon">gaps</span>
        </div>
    </div>
</div>
<script>
	CANVAS_SIZE = 4;
	$("#paper_size").keyup(function() {CANVAS_SIZE = (document.getElementById('paper_size').value); updateCanvas();});
	$("#paper_size").change(function() {CANVAS_SIZE = (document.getElementById('paper_size').value); updateCanvas();});
</script>