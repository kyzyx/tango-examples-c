package com.projecttango.experiments.rgbdepthsync;

import android.app.Activity;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.DialogInterface;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;

/**
 * Created by zyxlyr on 10/6/15.
 */
public class AdfSelectDialog extends DialogFragment {
    public AdfSelectDialog() {

    }
    public interface AdfSelectDialogListener {
        public void onAdfSelected(String s);
        public void onAdfCancel();
    }

    AdfSelectDialogListener listener;
    String[] adfs;
    String[] uuids;

    public void setAdfstring(String adfstring) {
        String[] rawadfs = adfstring.split(";");
        adfs = new String[rawadfs.length];
        uuids = new String[rawadfs.length];
        for (int i = 0; i < rawadfs.length; i++) {
            String[] raw = rawadfs[i].split(":");
            adfs[i] = raw[0];
            uuids[i] = raw[1];
        }
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            listener = (AdfSelectDialogListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()+ " must implement FilenameSelectDialogListener");
        }
    }
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        android.app.AlertDialog.Builder builder = new android.app.AlertDialog.Builder(getActivity());
        builder.setTitle("Pick an ADF")
                .setItems(adfs, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        listener.onAdfSelected(uuids[i]);
                        AdfSelectDialog.this.getDialog().dismiss();
                    }
                })
                .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        listener.onAdfCancel();
                        AdfSelectDialog.this.getDialog().cancel();
                    }
                });
        return builder.create();
    }
}

