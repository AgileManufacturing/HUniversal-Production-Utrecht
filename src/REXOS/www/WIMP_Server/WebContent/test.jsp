<%@page 
import="wsig.WSIGConfiguration,
		wsig.soap.SoapClient"
%>


<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
    "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" lang="en-US" xml:lang="en-US">
<head>
<title>.: WSIG Console :.</title>
<link rel="stylesheet" href="wsig.css"/>
</head>
<body>
<div class="nav" align="right"><font size="-2"><a href="http://jade.tilab.com/" target="_top">Jade - Java Agent DEvelopment Framework</a></font></div>
<h1>.: WSIG Console :.</h1>
<h3> <a href="index.jsp" class="title">Home</a> - <a href="test.jsp" class="title">Test</a></h3>

<%
	// Get configuration
	WSIGConfiguration wsigConfig = (WSIGConfiguration)application.getAttribute("WSIGConfiguration");

	// Get parameters
	String SOAPUrl = request.getParameter("soapurl");
	if (SOAPUrl == null) {
		SOAPUrl = wsigConfig.getWsigUri();
	}

	String SOAPRequest = request.getParameter("soaprequest");
	if (SOAPRequest == null) {
		SOAPRequest = "";
	}

	String SOAPResponse = "";
	if (SOAPRequest != null && !"".equals(SOAPRequest)) {
		SOAPResponse = SoapClient.sendStringMessage(SOAPUrl, SOAPRequest);
	}

%>	

<form method="post" action="test.jsp">
	<table width="80%" border="1">
		<tr><td colspan="2" class="head"><h2>Test page</h2></td></tr>
		<tr>
			<td width="20%" class="title">WebService url:</td>
			<td class="value"><input type="text" name="soapurl" size="80" value="<% out.print(SOAPUrl); %>"></input></td>
		</tr>
		<tr>
			<td width="20%" class="title">SOAP request:</td>
			<td class="value"><textarea cols=70 rows=10 WRAP="physical" name="soaprequest"><% out.print(SOAPRequest); %></textarea></td>
		</tr>
		<tr>
			<td width="20%" class="title">SOAP response:</td>
			<td class="value"><textarea cols=70 rows=10 WRAP="physical" name="soapresponse"><% out.print(SOAPResponse); %></textarea></td>
		</tr>
		<tr>
			<td width="20%" class="title"></td>
			<td class="value"><input type="submit" value="Send"></input><button type="reset">Reset</button></td>
		</tr>
	</table>
</form>

</body>
</html>