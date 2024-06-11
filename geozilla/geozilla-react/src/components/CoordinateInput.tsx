import React, { useState } from 'react';
import { TextField, Button, Snackbar } from '@mui/material';

const CoordinateInput = () => {
    const [longitude, setLongitude] = useState('');
    const [latitude, setLatitude] = useState('');
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = () => {
        const coordinates = {
            longitude: longitude,
            latitude: latitude
        };

        fetch('/api/send-coordinates', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(coordinates)
        })
            .then(response => {
                if (response.ok) {
                    setSnackbarOpen(true);
                    setLongitude('');
                    setLatitude('');
                    setError('');
                } else {
                    throw new Error('Не удалось отправить координаты');
                }
            })
            .catch(error => {
                console.error(error);
                setError('Произошла ошибка при отправке координат');
            });
    };

    return (
        <div>
            <TextField
                label="Долгота"
                value={longitude}
                onChange={(e) => setLongitude(e.target.value)}
            />
            <TextField
                label="Широта"
                value={latitude}
                onChange={(e) => setLatitude(e.target.value)}
            />
            <Button variant="contained" onClick={handleSubmit}>
                Отправить координаты
            </Button>
            {error && <p style={{ color: 'red' }}>{error}</p>}
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="Координаты успешно отправлены"
            />
        </div>
    );
};

export default CoordinateInput;
