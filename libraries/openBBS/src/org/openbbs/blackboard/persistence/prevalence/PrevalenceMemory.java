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

import java.util.Iterator;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import org.openbbs.blackboard.Zone;
import org.openbbs.blackboard.ZoneSelector;
import org.openbbs.blackboard.persistence.BlackboardMemory;
import org.openbbs.blackboard.persistence.TransientMemory;
import org.openbbs.blackboard.persistence.snapshot.Snapshotter;

/**
 * A Blackboard memory implementation that is based on object prevalence.
 * Entries will be kept in memory but all changes to the memory will be
 * logged to a file. When the memory is restored, it will replay the logged
 * changes and thereby restore the contents of the memory.
 */
public class PrevalenceMemory implements BlackboardMemory
{
   private TransientMemory memory = new TransientMemory();
   private Snapshotter snapshotter = null;
   private LogFile logFile;
   private boolean locked = false;

   /**
    * Create a new PrevalenceMemory. You have to set at least a
    * logFile before entries can be stored in a memory. If you
    * want support for memory snapshots, you have set a Snapshotter
    * too.
    */
   public PrevalenceMemory() {
      return;
   }

   /**
    * The logfile is the place where changes to the memory are logged.
    */
   public void setLogFile(LogFile logFile) {
      Validate.notNull(logFile, "logFile must be specified");

      if (this.logFile != null) {
         this.logFile.closeLog();
      }

      this.logFile = logFile;
   }

   /**
    * Set the Snapshotter to be used to create snapshots of this
    * memory.
    */
   public void setSnapshotter(Snapshotter snapshotter) {
      this.snapshotter = snapshotter;
   }

   /**
    * Restore the contents of this memory from the logfile. If a snapshotter
    * is set, the memory is restored from the latest snapshot before the logfile
    * is replayed. The entire memory is deleted first. If this method fails, the
    * state of the memory is undefined.
    * <p />
    * The memory will be locked while the restore is in progress. Attempts
    * to concurrently read or modify the memory will result in a
    * MemoryLockedException
    */
   public void restore() {
      Validate.notNull(this.logFile, "logFile is not set");

      this.checkLocked();
      this.lock();

      try {
         this.memory.clear();

         if (this.snapshotter != null) {
            this.snapshotter.restoreFromSnapshot(this.memory);
         }

         this.logFile.playback(new PlaybackDelegate() {
            public void createZone(Zone zone) {
               memory.createZone(zone);
            }

            public void dropZone(Zone zone) {
               memory.dropZone(zone);
            }

            public void storeEntry(Zone zone, Object entry) {
               memory.storeEntry(zone, entry);
            }

            public void removeEntry(Object entry) {
               memory.removeEntry(entry);
            }
         });
      }
      finally {
         this.unlock();
      }
   }

   /**
    * Store a snapshot of the memory's current state and reset the
    * logfile. The Snapshotter property must be set in order for
    * this method to work.
    * <p />
    * The memory will be locked while the snapshot is taken. Attempts
    * to concurrently read or modify the memory will result in a
    * MemoryLockedException
    */
   public void snapshot() {
      if (this.snapshotter == null) {
         throw new IllegalStateException("no Snapshotter defined for this memory");
      }

      this.checkLocked();
      this.lock();

      try {
         this.snapshotter.takeSnapshot(this.memory);
         this.logFile.reset();
      }
      finally {
         this.unlock();
      }
   }

   /**
    * Prevent reading and writing to the memory. If you try to
    * read or modify a locked memory, a MemoryLockedException
    * is thrown.
    */
   public void lock() {
      if (this.locked) {
         throw new IllegalStateException("memory is already locked");
      }
      this.locked = true;
   }

   /**
    * Unlock the previously locked memory.
    */
   public void unlock() {
      if (!this.locked) {
         throw new IllegalStateException("memory is not locked");
      }
      this.locked = false;
   }

   /**
    * Is the memory currently locked?
    */
   public boolean isLocked() {
      return this.locked;
   }

   /**
    * @see BlackboardMemory#createZone(Zone)
    */
   public void createZone(Zone zone) {
      this.checkLocked();
      this.memory.createZone(zone);
      this.logFile.writeCommand(new CreateZoneCommand(zone));
   }

   /**
    * @see BlackboardMemory#dropZone(Zone)
    */
   public void dropZone(Zone zone) {
      this.checkLocked();
      this.memory.dropZone(zone);
      this.logFile.writeCommand(new DropZoneCommand(zone));
   }

   /**
    * @see BlackboardMemory#entryExists(Object)
    */
   public boolean entryExists(Object entry) {
      this.checkLocked();
      return this.memory.entryExists(entry);
   }

   /**
    * @see BlackboardMemory#getEntries(ZoneSelector, EntryFilter)
    */
   public Iterator<Object> getEntries(ZoneSelector zoneSelector, EntryFilter entryFilter) {
      this.checkLocked();
      return this.memory.getEntries(zoneSelector, entryFilter);
   }

   /**
    * @see BlackboardMemory#getZone(Object)
    */
   public Zone getZone(Object entry) {
      this.checkLocked();
      return this.memory.getZone(entry);
   }

   /**
    * @see BlackboardMemory#removeEntry(Object)
    */
   public void removeEntry(Object entry) {
      this.checkLocked();
      this.memory.removeEntry(entry);
      this.logFile.writeCommand(new RemoveEntryCommand(entry));
   }

   /**
    * @see BlackboardMemory#storeEntry(Zone, Object)
    */
   public void storeEntry(Zone zone, Object entry) {
      this.checkLocked();
      this.memory.storeEntry(zone, entry);
      this.logFile.writeCommand(new StoreEntryCommand(zone, entry));
   }

   /**
    * @see BlackboardMemory#zoneExists(Zone)
    */
   public boolean zoneExists(Zone zone) {
      this.checkLocked();
      return this.memory.zoneExists(zone);
   }

   /**
    * Throw a MemoryLockedException if the memory is locked.
    */
   public void checkLocked() {
      if (this.locked) {
         throw new MemoryLockedException("memory is currently locked, try again later");
      }
   }
}
