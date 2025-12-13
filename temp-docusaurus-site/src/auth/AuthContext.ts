import React from 'react';
import { User } from 'firebase/auth';

export interface AuthContextType {
  user: User | null;
  loading: boolean;
}

export const AuthContext = React.createContext<AuthContextType>({
  user: null,
  loading: true,
});
