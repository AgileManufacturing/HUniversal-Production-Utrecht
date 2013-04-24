/**
 * @author Ammar Abdulamir
 * @file BlackboardReader.java
 * @brief A blackboard reader GUI.
 * @date Created: 4/9/13
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
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
package tools.blackboard_reader;

import com.mongodb.BasicDBObject;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import tools.blackboard_reader.models.Blackboards;
import tools.blackboard_reader.models.Entries;
import tools.blackboard_reader.models.EntryNode;

import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.util.ArrayList;

/**
 * A blackboard reader GUI.
 **/
public class BlackboardReader implements ListSelectionListener {
    /**
     * The main panel of the form.
     *
     * @var JPanel mainPanel
     **/
    private JPanel mainPanel;
    /**
     * A list of blackboards.
     *
     * @var JList collections
     **/
    private JList collections;
    /**
     * The tree containing entries of the currently selected blackboard.
     *
     * @var JTree entries
     **/
    private JTree entries;
    /**
     * A list of blackboard entries.
     *
     * @var ArrayList blackboardEntries
     **/
    private final ArrayList<EntryNode> blackboardEntries;
    /**
     * The blackboards.
     *
     * @var Blackboard blackboard
     **/
    private final Blackboards blackboards;

    /**
     * BlackboardReader constructor.
     **/
    public BlackboardReader() {
        JFrame frame = new JFrame("Blackboard Reader");
        frame.setContentPane(mainPanel);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);

        blackboardEntries = new ArrayList<EntryNode>();
        blackboards = new Blackboards();
        collections.addListSelectionListener(this);

    }

    /**
     * Used to get the blackboards(collections).
     *
     * @return Returns blackboards(collections).
     */
    public Blackboards getBlackboards() {
        return blackboards;
    }

    /**
     * Updates the model for the current blackboard.
     *
     * @param collection A blackboard.
     **/
    public void updateEntriesModel(DBCollection collection) {
        Entries model = new Entries();

        DBCursor cursor = collection.find();
        while (cursor.hasNext()) {
            model.add(new EntryNode((BasicDBObject) cursor.next()));
        }

        entries.setModel(model);
    }

    /**
     * Update the blackboards list.
     **/
    public void updateBlackboards() {
        collections.setModel(blackboards);
    }

    /**
     * Gets the entries of the currently selected blackboard.
     *
     * @return Returns the entries for the currently selected blackboard.
     **/
    public ArrayList<EntryNode> getBlackboardEntries() {
        return blackboardEntries;
    }

    /**
     * Triggered when a blackboard has been selected in the list.
     *
     * @param e A ListSelectionEvent.
     */
    @Override
    public void valueChanged(ListSelectionEvent e) {
        if (!e.getValueIsAdjusting()) {
            updateEntriesModel((DBCollection) collections.getModel().getElementAt(collections.getSelectedIndex()));
        }
    }
}