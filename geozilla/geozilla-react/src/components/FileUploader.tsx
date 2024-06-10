import React, { useState, ChangeEvent } from 'react';
import { Button, Snackbar } from '@mui/material';

const FileUploader = () => {
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [uploadedFile, setUploadedFile] = useState<File | null>(null);

    const handleFileUpload = (event: ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            setSnackbarOpen(true);
            setUploadedFile(file);
        }
    };

    return (
        <div>
            <input
                accept=".b3dm"
                style={{ display: 'none' }}
                id="raised-button-file"
                type="file"
                onChange={handleFileUpload}
            />
            <label htmlFor="raised-button-file">
                <Button variant="contained" component="span">
                    Upload B3DM File
                </Button>
            </label>
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="File uploaded successfully"
            />
            <div>
                {uploadedFile?.name} - {uploadedFile?.size}
            </div>
        </div>
    );
};

export default FileUploader;
