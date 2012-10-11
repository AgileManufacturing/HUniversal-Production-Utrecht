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

/**
 * A LogFile stores {@link PrevalenceCommands} exactly in the order in
 * which they are written. Thereby a LogFile can record the changes which
 * are made to a PrevalenceMemory. Later, the commands in a LogFile can
 * be replayed to restore the state of a memory.
 */
public interface LogFile
{
   /**
    * Playback all commands which are currently stored in this LogFile
    * in the order in which they were written.
    * 
    * @param playbackDelegate  a non-null delegate object that will be
    *                          passed to each command.
    */
   public void playback(PlaybackDelegate playbackDelegate);

   /**
    * Write a command to the LogFile. It is important that the command
    * is appended at the end of the file. If the LogFile is not open,
    * it will be opened implicitly and remain open until it received a
    * closeLog message or until the system terminates.
    * 
    * @param command  the command to be stored in the LogFile; not null.
    */
   public void writeCommand(PrevalenceCommand command);

   /**
    * Remove previously stored commands from the LogFile. The logfile
    * must not be modified before this method returns. The caller is
    * responsbible for ensuring this.
    */
   public void reset();

   /**
    * Close this LogFile. Commands which are not persisted yet must be
    * written. Allocated resources have to be freed. The LogFile
    * can still receive writeCommand messages after it has been closed
    * and the commands will be stored in the log. If the LogFile is
    * already closed this method does nothing.
    */
   public void closeLog();

   /**
    * Test if the LogFile is currently open.
    */
   public boolean isOpen();
}
