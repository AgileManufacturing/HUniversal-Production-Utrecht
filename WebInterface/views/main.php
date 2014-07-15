<div id="header">
	<img src="images/HU_logo.png"/>
    <ul id="header_container">
    	<li>
        	<a href="#">Support</a>
        </li>
    	<li>
        	<a href="http://wiki.agilemanufacturing.nl/index.php/Main_Page">Documentation</a>
        </li>
    </ul>
</div>
<div id="nav">
	<img src="images/REXOS_logo.png" class="logo" />
    <ul>
    	<a href="./">
        	<li <?php if(!isset($_REQUEST['page'])) echo 'class="selected"' ?>>
            	Dashboard
            </li>
        </a>
    	<a href="./Services">
        	<li <?php if(isset($_REQUEST['page']) && $_REQUEST['page']=='Services') echo 'class="selected"' ?>>
        		Services
            </li>
        </a>
    </ul>
</div>
<div id="content">
	<?php
        $page = 'dashboard';
        if (isset($_REQUEST['page'])){
            $page = strtolower($_REQUEST['page']);
        }
        $url = 'views/' . $page . '/' . $page . '.php';
        if (file_exists ( $url )){
            include($url);
        }
		else {
			include('views/404.php');
		}
    ?>	
</div>
<ul id="notificator" class="list-group" style="position:fixed; width:50%; bottom:-20px; right:0px;"></ul>