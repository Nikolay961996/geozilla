import React, { useState, ChangeEvent } from 'react';
import { Button, Snackbar } from '@mui/material';

interface FileUploaderProps {
    setUploadedFile: React.Dispatch<React.SetStateAction<File | null>>;
}

const FileUploader: React.FC<FileUploaderProps> = ({setUploadedFile}) => {
    const [snackbarOpen, setSnackbarOpen] = useState(false);

    const handleFileUpload = async (event: ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            setSnackbarOpen(true);
            setUploadedFile(file);
        }
    };

    return (
        <div>
            <input
                accept=".b3dm, .json"
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
        </div>
    );
};

export default FileUploader;
