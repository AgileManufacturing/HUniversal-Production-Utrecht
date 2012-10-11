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

import java.io.Serializable;

/**
 * A PrevalenceCommands represents a change to a PrevalenceMemory. By
 * playing back a set of stored commands, changes can be restored.
 */
public interface PrevalenceCommand extends Serializable
{
   /**
    * Redo the change represented by this command. The command
    * has to send the appropriate message to the specified
    * PlaybackDelegate.
    */
   public void playback(PlaybackDelegate playbackDelegate);
}
