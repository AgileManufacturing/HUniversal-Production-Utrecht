/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig.wsdl;

import jade.content.onto.BasicOntology;

import java.util.Hashtable;

public class WSDLConstants {
	
	// WSDL Constants
	public static final String STYLE_RPC = "rpc";
	public static final String STYLE_DOCUMENT = "document";
	public static final String USE_ENCODED = "encoded";
	public static final String USE_LITERAL = "literal";
	public static final String URN = "urn";
	public static final String XSI = "xsi";
	public static final String XSI_URL = "http://www.w3.org/2001/XMLSchema-instance";
	public static final String XSD = "xsd";
	public static final String XSD_URL = "http://www.w3.org/2001/XMLSchema";
	public static final String WSDL_SOAP = "wsdlsoap";
	public static final String WSDL_SOAP_URL = "http://schemas.xmlsoap.org/wsdl/soap/";
	public static final String ENCODING_URL = "http://schemas.xmlsoap.org/soap/encoding/";
	public static final String TRANSPORT_URL = "http://schemas.xmlsoap.org/soap/http";
	public static final String SOAPENVELOP_PREFIX = "soapenv";
	public static final String BINDING_SUFFIX = "Binding";
	public static final String PORT_TYPE_SUFFIX = "Port";
	public static final String SERVICE_SUFFIX = "Service";
	public static final String ACTION_SUFFIX = "Action";
	public static final String REQUEST_SUFFIX = "Request";
	public static final String RESPONSE_SUFFIX = "Response";
	public static final String RETURN_SUFFIX = "Return";
	public static final String SCHEMA = "schema";
	public static final String OPERATION = "operation";
	public static final String BODY = "body";
	public static final String ARRAY_OF = "ArrayOf";
	public static final String BINDING = "binding";
	public static final String PARAMETERS = "parameters";
	public static final String TYPE = "type";
	public static final String MAPPER_METHOD_PREFIX = "to";
	public static final String SEPARATOR = "_";
	
	// XSD primitive types
	public static final String XSD_STRING = "string";
	public static final String XSD_BOOLEAN = "boolean";
	public static final String XSD_INT = "int";
	public static final String XSD_LONG = "long";
	public static final String XSD_FLOAT = "float";
	public static final String XSD_DOUBLE = "double";
	public static final String XSD_DATETIME = "dateTime";
	public static final String XSD_BYTE = "byte";
	public static final String XSD_SHORT = "short";
	
	// Primitive conversion types from jade to xsd (used for ontologies)
	public static final Hashtable<String, String> jade2xsd = new Hashtable<String, String>();
	static {
		jade2xsd.put(BasicOntology.FLOAT, XSD_FLOAT);
		jade2xsd.put(BasicOntology.INTEGER, XSD_INT);
		jade2xsd.put(BasicOntology.STRING, XSD_STRING);
		jade2xsd.put(BasicOntology.BOOLEAN, XSD_BOOLEAN);
		jade2xsd.put(BasicOntology.DATE, XSD_DATETIME);
		jade2xsd.put(BasicOntology.BYTE_SEQUENCE, XSD_BYTE);
	}

	// Primitive conversion types from java to xsd (used for mapper)	
	public static final Hashtable<Class, String> java2xsd = new Hashtable<Class, String>();
	static {
		java2xsd.put(java.lang.String.class, XSD_STRING);
		java2xsd.put(java.lang.Boolean.class, XSD_BOOLEAN);
		java2xsd.put(java.lang.Byte.class, XSD_BYTE);
		java2xsd.put(java.lang.Double.class, XSD_DOUBLE);
		java2xsd.put(java.lang.Float.class, XSD_FLOAT);
		java2xsd.put(java.lang.Integer.class, XSD_INT);
		java2xsd.put(java.lang.Long.class, XSD_LONG);
		java2xsd.put(java.lang.Short.class, XSD_SHORT);
		java2xsd.put(java.util.Calendar.class, XSD_DATETIME);
		java2xsd.put(java.util.Date.class, XSD_DATETIME);
	}
}
