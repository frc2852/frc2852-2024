package frc.robot.util;

public class NoteTracker {
    private boolean hasNote;

    public boolean hasNote() {
        return hasNote;
    }

    public void setNoteShot() {
        this.hasNote = false;
    }

    public void setNoteAcquired() {
        this.hasNote = true;
    }
}
