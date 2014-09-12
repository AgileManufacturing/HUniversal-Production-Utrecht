<div class="form-group">
    <label for="service_selector" class="col-sm-6 control-label" style="text-align:left">
        Color:
    </label>
    <div class="col-sm-6">
        <select id="ball_color" class="form-control">
            <option value="#c00">red</option>
            <option value="#00c">blue</option>
            <option value="#cc0">yellow</option>
        </select>
    </div>
</div>

<div class="form-group">
    <label for="service_selector" class="col-sm-6 control-label" style="text-align:left">
        Ball size:
    </label>
    <div class="col-sm-6">
        <div class="input-group">
            <input id="" type="number" class="form-control" step="any" value="9">
            <span class="input-group-addon">mm</span>
        </div>
    </div>
</div>
<script>
	var subjects = new Array();
	subjects.identifier = 'ball';
	subjects.color = $("#ball_color option:selected").text();
	productStepHandler.setProperty("subjects",subjects);
	
	$("#ball_color").change(function() {
		var ball_color_selector = document.getElementById('ball_color');
        var color = ball_color_selector.options[ball_color_selector.selectedIndex].value;
		SQUARE_COLOR = color;
		
		var subjects = new Array();
		subjects.identifier = 'ball';
		subjects.color = $("#ball_color option:selected").text();
		productStepHandler.setProperty("subjects",subjects);
	});
</script>