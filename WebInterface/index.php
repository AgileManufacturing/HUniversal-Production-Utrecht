<?php	
	require_once(__DIR__.'/lang/Languages.class.php');
	require_once(__DIR__.'/canvases/Canvas.class.php');
?>

<!DOCTYPE html>
<html lang="en">
	<head>
		<meta charset="utf-8"/>
		<title>WIMP</title>
        <script src="http://code.jquery.com/jquery-latest.js"></script>
        <script src="js/deepCopy.js"></script>
        <script src="js/ProductStep.js"></script>
        <script src="js/ProductStepHandler.js"></script>
        <script src="websocket/connection.js"></script>
		<meta name="viewport" content="width=device-width, minimum-scale=1.0, maximum-scale=1.0" />
		<link rel="stylesheet" href="//netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css">
		<link href="css/main.css" type="text/css" rel="stylesheet">
	</head>
	<body>
		<?php
            include('views/main.php');
        ?>
	</body>
</html>