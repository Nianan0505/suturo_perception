package de.suturo.suturospeechrecognizer.suturospeechrecognizer;

// Interface so the HTTP Asyntask can trigger a callback in the Activity
public interface OnTaskCompleted {
    void onTaskCompleted(Boolean result);
}
