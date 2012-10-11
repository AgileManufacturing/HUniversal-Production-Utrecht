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
package org.openbbs.blackboard.persistence.snapshot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.Iterator;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.Zone;
import org.openbbs.util.ObjectReader;

/**
 * A Snapshotter takes a Snapshottable BlackboardMemory and dumps
 * its contents to a file. 
 */
public class Snapshotter
{
   private File snapshotFile;

   /**
    * Create a new Snapshotter. You have to set the snapshotFile before
    * you can take snapshots or restore SnapshottableMemories from a
    * snapshot.
    */
   public Snapshotter() {
      return;
   }

   /**
    * Create a new Snapshotter and set the snapshotFile.
    */
   public Snapshotter(File snapshotFile) {
      this.setSnapshotFile(snapshotFile);
   }

   /**
    * Set the snapshot file. Each time a new snapshot is taken, the
    * contents of the file are replaced with the contents of the
    * snapshotted memory.
    */
   public void setSnapshotFile(File snapshotFile) {
      Validate.notNull(snapshotFile);
      this.snapshotFile = snapshotFile;
   }

   /**
    * Save the contents of a {@link SnapshottableMemory} to the snapshot
    * file. The memory must not be modified while taking the snapshot.
    * 
    * @param memory  a non-null SnapshottableMemory
    */
   public void takeSnapshot(SnapshottableMemory memory) {
      Validate.notNull(this.snapshotFile, "snapshot file not set");

      try {
         this.basicTakeSnapshot(memory);
      }
      catch (IOException exc) {
         throw new SnapshotException("failed to create snapshot (file=" + this.snapshotFile + ")", exc);
      }
   }

   /**
    * Restore the contents of {@link SnapshottleMemory} from a snapshot
    * file. It is up to the memory to clear its contents before or to
    * handle conflicts otherwise. The memory must not be modified while
    * its contents are restored. Does nothing if the snapshotFile does
    * not exist or is emtpy, this method 
    * 
    * @param memory  a non-null SnapshottableMemory
    */
   public void restoreFromSnapshot(SnapshottableMemory memory) {
      Validate.notNull(this.snapshotFile, "snapshotFile not set");

      if (!this.snapshotFile.exists()) {
         return;
      }

      try {
         this.basicRestoreFromSnapshot(memory);
      }
      catch (Exception exc) {
         throw new SnapshotException("failed to restore contents of snapshot file " + this.snapshotFile, exc);
      }
   }

   private void basicTakeSnapshot(SnapshottableMemory memory) throws IOException {
      Validate.notNull(memory, "memory is null");
      Validate.notNull(this.snapshotFile, "snapshotFile is not set");

      ObjectOutputStream ostream = new ObjectOutputStream(new FileOutputStream(this.snapshotFile));
      try {
         for (Iterator<Zone> zoneIt = memory.getZonesIterator(); zoneIt.hasNext();) {
            ostream.writeObject(SnapshotMarker.NEW_ZONE);
            Zone zone = zoneIt.next();
            ostream.writeObject(zone);

            Iterator<Object> entryIt = memory.getEntriesInZone(zone);
            if (entryIt != null) {
               while (entryIt.hasNext()) {
                  ostream.writeObject(entryIt.next());
               }
            }
         }
      }
      finally {
         ostream.close();
      }
   }

   private void basicRestoreFromSnapshot(SnapshottableMemory memory) throws Exception {
      Validate.notNull(memory, "memory is null");
      Validate.notNull(this.snapshotFile, "snapshotFile is not set");

      final SnapshottableMemory _memory = memory;
      new ObjectReader(this.snapshotFile).readObjects(new ObjectReader.Delegate() {
         Zone currentlyRestoredZone = null;

         public void didReadObject(Object object) throws Exception {
            if (SnapshotMarker.NEW_ZONE.equals(object)) {
               // force the next object to be a zone
               this.currentlyRestoredZone = null;
            }
            else if (currentlyRestoredZone == null) {
               Validate.isTrue(object instanceof Zone, object + " is not a Zone but should be");
               _memory.restoreZone((Zone)object);
               this.currentlyRestoredZone = (Zone)object;
            }
            else {
               Validate.notNull(this.currentlyRestoredZone, "entry " + object + " found outside a zone");
               _memory.restoreEntry(object, this.currentlyRestoredZone);
            }
         }
      });
   }
}
