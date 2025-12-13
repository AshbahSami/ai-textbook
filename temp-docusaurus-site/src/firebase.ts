// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getAuth } from "firebase/auth";
import { getFirestore } from "firebase/firestore";

// TODO: Add your Firebase project configuration
// See https://firebase.google.com/docs/web/setup#available-libraries
const firebaseConfig = {
  apiKey: "AIzaSyB34gZp4Zt5TqxkuFRfIVR45M7B14vKGH4",
  authDomain: "hackathon-book.firebaseapp.com",
  projectId: "hackathon-book",
  storageBucket: "hackathon-book.firebasestorage.app",
  messagingSenderId: "440365410240",
  appId: "1:440365410240:web:b46a25fdd40eb7cb900a25",
  measurementId: "G-9XBLDHDZKC"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const auth = getAuth(app);
const db = getFirestore(app);

export { auth, db };
