#include <xml_io/xml.h>
#include <string>
#include "tinyxml.h" 

namespace utility_xml {
	/* This function retrieves data from an xml file
	 *
 	 * @param char* $path
 	 * This parameter specifies the path of the xml file
 	 */
	void crate_data(char* path, std::string crate_name, double* crate_width = NULL, double* crate_height = NULL, 
						double* crate_depth = NULL, int* col_nr = NULL, int* row_nr = NULL, double* distance_to_side = NULL, 
						double* distance_to_next = NULL, double* container_radius = NULL, double* bottom_thickness = NULL){

		int i = crate_name.size();
		while (isdigit(crate_name[i - 1])) {
			i--;
		}

		char* crate_serie = new char[crate_name.size() + 1];
		strcpy(crate_serie, crate_name.c_str());

		crate_serie[i] = '\0';

		TiXmlDocument doc(path);
		if ( !doc.LoadFile() ) return;

		TiXmlElement* crate = doc.FirstChildElement()->FirstChildElement();

		while(1){
			if (strcmp(crate->FirstAttribute()->ValueStr().c_str(), crate_serie) == 0) break;

			crate = crate->NextSiblingElement();
			if (crate == NULL) break;
		}

		if (crate == NULL) return;
		
		if (crate_width != NULL) *crate_width = strtod(crate->FirstChildElement("crate_width")->GetText(), NULL);
		if (crate_height != NULL) *crate_height = strtod(crate->FirstChildElement("crate_height")->GetText(), NULL);
		if (crate_depth != NULL) *crate_depth = strtod(crate->FirstChildElement("crate_depth")->GetText(), NULL);
		
		if (col_nr != NULL) *col_nr = atoi(crate->FirstChildElement("grid_width")->GetText());
		if (row_nr != NULL) *row_nr = atoi(crate->FirstChildElement("grid_width")->GetText());
		
		if (distance_to_side != NULL) *distance_to_side = strtod(crate->FirstChildElement("distance_to_side")->GetText(), NULL);
		if (distance_to_next != NULL) *distance_to_next = strtod(crate->FirstChildElement("distance_to_next")->GetText(), NULL);
		
		if (container_radius != NULL) *container_radius = strtod(crate->FirstChildElement("radius_of_ball_container")->GetText(), NULL);
		if (bottom_thickness != NULL) *bottom_thickness = strtod(crate->FirstChildElement("bottom_thickness")->GetText(), NULL);
	}
}
