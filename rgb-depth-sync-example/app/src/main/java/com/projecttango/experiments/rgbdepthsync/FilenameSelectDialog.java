package com.projecttango.experiments.rgbdepthsync;

import android.app.Activity;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.DialogInterface;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.EditText;

public class FilenameSelectDialog extends DialogFragment {
    public FilenameSelectDialog() {

    }
    public interface FilenameSelectDialogListener {
        void onPositiveClick(String s);
        void onNegativeClick(String s);
    }

    FilenameSelectDialogListener listener;

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            listener = (FilenameSelectDialogListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()+ " must implement FilenameSelectDialogListener");
        }
    }
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        android.app.AlertDialog.Builder builder = new android.app.AlertDialog.Builder(getActivity());
        LayoutInflater inflater = getActivity().getLayoutInflater();
        final View v = inflater.inflate(R.layout.dialog_savefile, null);
        builder.setView(v)
                .setPositiveButton("Start Capture", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        EditText filenametext = (EditText) v.findViewById(R.id.filenameText);
                        listener.onPositiveClick(filenametext.getText().toString());
                    }
                })
                .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        EditText filenametext = (EditText) v.findViewById(R.id.filenameText);
                        listener.onNegativeClick(filenametext.getText().toString());
                        FilenameSelectDialog.this.getDialog().cancel();
                    }
                });
        return builder.create();
    }
}
