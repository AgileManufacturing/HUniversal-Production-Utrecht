/*
 *  Copyright [2006-2007] [Stefan Kleine Stegemann]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package org.openbbs.blackboard.persistence.prevalence;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import org.apache.commons.lang.Validate;
import org.openbbs.util.ObjectReader;

/**
 * A LogFile implementation which writes all commands to a file
 * on the local filesystem.
 */
public class SimpleLogFile implements LogFile
{
   private ObjectOutputStream logStream = null;
   private File outputFile = null;

   /**
    * Create a new LogFile and set the ouput file to the specified
    * file.
    */
   public SimpleLogFile(File outputFile) {
      this.setOutputFile(outputFile);
   }

   /**
    * Create new LogFile. You have to set the outputFile before
    * the LogFile can be used.
    */
   public SimpleLogFile() {
      return;
   }

   /**
    * Set the output file where the commands that are written to this
    * LogFile will be stored.
    */
   public void setOutputFile(File outputFile) {
      Validate.notNull(outputFile, "outputFile must not be null");

      if (this.isOpen()) {
         this.closeLog();
      }

      this.outputFile = outputFile;
   }

   /**
    * @see LogFile#playback(PlaybackDelegate)
    */
   public void playback(PlaybackDelegate playbackDelegate) {
      Validate.notNull(playbackDelegate, "playbackDelegate is null");

      if (this.isOpen()) {
         this.closeLog();
      }

      try {
         final PlaybackDelegate _playbackDelegate = playbackDelegate;
         new ObjectReader(this.outputFile).readObjects(new ObjectReader.Delegate() {
            public void didReadObject(Object object) throws Exception {
               ((PrevalenceCommand)object).playback(_playbackDelegate);
            }
         });
      }
      catch (Exception exc) {
         throw new PrevalencePersistenceException("failed to restore entries from output file " + this.outputFile, exc);
      }
   }

   /**
    * @see LogFile#writeCommand(PrevalenceCommand)
    */
   public void writeCommand(PrevalenceCommand command) {
      Validate.notNull(command);

      if (!this.isOpen()) {
         this.openLog();
      }

      try {
         this.logStream.writeObject(command);
         this.logStream.flush();
      }
      catch (IOException exc) {
         throw new LogFileException("failed to write command " + command + " to " + this.outputFile, exc);
      }
   }

   /**
    * @see LogFile#reset()
    */
   public void reset() {
      this.closeLog();
      if (this.outputFile.exists()) {
         Validate.isTrue(this.outputFile.delete(), "cannot remove log file " + this.outputFile);
      }
   }

   /**
    * @see LogFile#closeLog()
    */
   public void closeLog() {
      if (!this.isOpen()) return; // not open

      try {
         this.logStream.flush();
         this.logStream.close();
         this.logStream = null;
      }
      catch (IOException exc) {
         throw new LogFileException("failed to close output file " + this.outputFile, exc);
      }
   }

   /**
    * Test if the logfile is currently open.
    */
   public boolean isOpen() {
      return this.logStream != null;
   }

   /**
    * Open the file for writing.
    */
   private void openLog() {
      Validate.isTrue(!this.isOpen(), "command stream is already open");
      Validate.notNull(this.outputFile, "outputFile is not set");

      try {
         // Appending to an ObjectOutputStream is NOT possible. If the output
         // file exists, we have to copy all objects stored in this file to the
         // new stream.
         File oldOutputFile = null;
         if (this.outputFile.exists()) {
            oldOutputFile = new File(this.outputFile.getAbsolutePath() + "_old");
            Validate.isTrue(this.outputFile.renameTo(oldOutputFile), "cannot rename" + this.outputFile + " to "
                     + oldOutputFile);
         }

         this.logStream = new ObjectOutputStream(new FileOutputStream(this.outputFile, false));

         if (oldOutputFile != null) {
            new ObjectReader(oldOutputFile).readObjects(new ObjectReader.Delegate() {
               public void didReadObject(Object object) throws Exception {
                  logStream.writeObject(object);
               }
            });
            oldOutputFile.delete();
         }
      }
      catch (Exception exc) {
         this.logStream = null;
         throw new LogFileException("unable to open logFile " + this.outputFile, exc);
      }
   }
}
