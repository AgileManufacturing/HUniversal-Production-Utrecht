/**
 * @file StepperMotorProperties.cpp
 * @brief Contains the properties of a stepper motor.
 * @date Created: 2012-10-02
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
 * License: newBSD
 * 
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <rexos_zip/ZipExtractor.h>

#include <unistd.h>
#include <zip.h>
#include <iostream>
#include <fstream>
#include <unistd.h>


namespace rexos_zip {
	ZipExtractor::ZipExtractor() {
	}
	void ZipExtractor::extractZipArchive(std::istream* inputFile, std::string archiveBasename, boost::filesystem::path targetDir) {
		char buf[100];
		
		// write the zip archive to the file system
		if(boost::filesystem::exists(targetDir) == false) {
			boost::filesystem::create_directories(targetDir);
		}
		
		std::ofstream zipFileOutputStream;
		zipFileOutputStream.open(targetDir.string() + archiveBasename + ".zip", 
				std::ios::out | std::ios::binary); 
		
		while(inputFile->good() == true) {
			inputFile->read(buf, sizeof(buf));
			zipFileOutputStream.write(buf, inputFile->gcount());
		}
		zipFileOutputStream.close();
		REXOS_DEBUG_STREAM("zip archive has been written at " << targetDir.string() + archiveBasename + ".zip");
		
		// extract the zip archive
		int err = 0;
		zip* zipArchive = zip_open((targetDir.string() + archiveBasename + ".zip").c_str(), 0, &err);
		if(err != 0 || zipArchive == NULL) {
			REXOS_ERROR_STREAM("zip archive opened with " << err);
			return;
		}
		
		// create the target directory
		boost::filesystem::path extractionPath = targetDir.string() + archiveBasename + "/";
		boost::filesystem::create_directories(extractionPath);
		
		struct zip_stat zipStat;
		for (int i = 0; i < zip_get_num_entries(zipArchive, 0); i++) {
			if (zip_stat_index(zipArchive, i, 0, &zipStat) == 0) {
				// is directory or file entry?
				if (zipStat.name[strlen(zipStat.name) - 1] == '/') {
					boost::filesystem::create_directories(extractionPath.string() + zipStat.name);
				} else {
					// files could be specified before the upper directories. create these directories
					boost::filesystem::path path = boost::filesystem::path(extractionPath.string() + zipStat.name);
					boost::filesystem::create_directories(path.parent_path());
					
					struct zip_file* zipFile;
					zipFile = zip_fopen_index(zipArchive, i, 0);
					if (!zipFile) {
						throw std::runtime_error("Unable to open zipFile in zipArchive");
					}
					
					std::ofstream outputFileStream;
					outputFileStream.open((extractionPath.string() + std::string(zipStat.name)).c_str(), std::ios::out | std::ios::binary); 
					if (outputFileStream.good() != true) {
						throw std::runtime_error("Unable to open fstream with path" + (extractionPath.string() + std::string(zipStat.name)));
					}
					
					int sum = 0;
					while (sum < zipStat.size) {
						int len = zip_fread(zipFile, buf, 100);
						outputFileStream.write(buf, len);
						sum += len;
					}
					
					outputFileStream.close();
					zip_fclose(zipFile);
				}
			}
		}
		REXOS_DEBUG("zipArchive has been extracted");
	}
}
